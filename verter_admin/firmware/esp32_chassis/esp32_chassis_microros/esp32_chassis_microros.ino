/**
 * ESP32 Chassis Controller — micro-ROS
 *
 * Publishes : /wheel_encoders (std_msgs/Int64MultiArray, BEST_EFFORT, 20 Hz)
 * Subscribes: /cmd_vel        (geometry_msgs/Twist,      RELIABLE)
 *
 * I2C hang mitigation
 * -------------------
 * Right encoder (Wire1) periodically blocks up to 1 s due to arduino-esp32
 * v2.x bug (Wire.setTimeout < 1000 ms has no effect, issue #5934).
 * Fix: encoder reads run in a dedicated FreeRTOS task on Core 0.
 * If Wire1 takes > 60 ms → bus reset, encoder marked stale, nothing published.
 * Core 1 (micro-ROS + PID) continues uninterrupted during the hang.
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <Wire.h>

// ============================================================================
// HARDWARE CONSTANTS
// ============================================================================

// Motors (Cytron MD10C: PWM + DIR)
#define L_PWM_PIN    19
#define L_DIR_PIN    27
#define R_PWM_PIN    18
#define R_DIR_PIN    25

// Motor forward direction levels
#define L_DIR_FWD    LOW    // LOW  = forward for left  motor
#define R_DIR_FWD    HIGH   // HIGH = forward for right motor

// PWM channels (ESP32 ledc)
#define L_PWM_CHAN   0
#define R_PWM_CHAN   1
#define PWM_FREQ_HZ  20000  // 20 kHz (inaudible, recommended for MD10C)
#define PWM_BITS     8      // 0-255
#define MAX_PWM      200
#define MIN_PWM      25     // dead-zone threshold

// Encoders (AS5600, 12-bit absolute magnetic)
#define L_ENC_SDA    21     // Left  encoder — I2C0 (Wire)
#define L_ENC_SCL    22
#define R_ENC_SDA    32     // Right encoder — I2C1 (Wire1)
#define R_ENC_SCL    4
#define AS5600_ADDR  0x36
#define AS5600_REG   0x0C   // RAW_ANGLE high byte (12-bit in 0x0C-0x0D)
#define ENC_RES      4096   // Steps per revolution
#define L_ENC_SIGN   (-1)   // Left  encoder counts DOWN when moving forward
#define R_ENC_SIGN   (+1)   // Right encoder counts UP  when moving forward

// Robot kinematics (calibrated)
#define WHEEL_CIRC   0.576f   // m
#define GEAR_RATIO   4.0007f
#define WHEEL_BASE   0.386f   // m
#define METERS_STEP  (WHEEL_CIRC / (ENC_RES * GEAR_RATIO))   // ~3.52e-5 m/step
#define MAX_VEL      0.5f     // m/s

// PID (velocity controller — input: m/s error, output: PWM units)
#define PID_KP          80.0f
#define PID_KI          50.0f
#define PID_KD           5.0f
#define PID_MAX_INTEG  100.0f
#define PID_INTERVAL_MS  50   // 20 Hz
#define MAX_PWM_CHANGE   15   // ramp limiter: max PWM delta per PID tick

// Timing / safety
#define CMD_TIMEOUT_MS    500   // ms without /cmd_vel  → stop motors
#define ENC_STALE_MS      150   // ms without encoder update → stale
#define I2C_HANG_THRESH    60   // ms: Wire1 read taking longer = hang

// LED
#define LED_PIN 2

// micro-ROS serial baud rate override (weak symbol from library)
extern "C" bool arduino_transport_open(struct uxrCustomTransport *) {
    Serial.begin(921600);
    return true;
}

// ============================================================================
// SHARED STATE — encoder task (Core 0) ↔ main loop (Core 1)
// ============================================================================

struct EncoderShared {
    int32_t left_steps;
    int32_t right_steps;
    float   left_vel;      // m/s
    float   right_vel;     // m/s
    bool    fresh;
    uint32_t last_ms;
};
static EncoderShared g_enc = {};
static SemaphoreHandle_t g_enc_mutex;

// cmd_vel — written only in micro-ROS callback (Core 1), read in loop (Core 1)
struct CmdVelState {
    float linear;
    float angular;
    uint32_t last_ms;
};
static CmdVelState g_cmd = {};

// ============================================================================
// PID CONTROLLER
// Implemented as a class: Arduino IDE does not auto-generate prototypes for
// class methods, avoiding ctags failures with user-defined types in signatures.
// ============================================================================

class PIDController {
public:
    float integral   = 0.0f;
    float last_error = 0.0f;
    int   last_pwm   = 0;

    // target / actual in m/s; dt in seconds; returns signed PWM
    int update(float target, float actual, float dt) {
        float error = target - actual;
        integral    = constrain(integral + error * dt, -PID_MAX_INTEG, PID_MAX_INTEG);
        float deriv = (dt > 0.0f) ? (error - last_error) / dt : 0.0f;
        last_error  = error;

        float raw_out = PID_KP * error + PID_KI * integral + PID_KD * deriv;
        int desired   = (int)constrain(raw_out, -MAX_PWM, MAX_PWM);

        // Ramp limiter: prevents step changes in PWM
        int pwm  = constrain(desired, last_pwm - MAX_PWM_CHANGE, last_pwm + MAX_PWM_CHANGE);
        last_pwm = pwm;
        return pwm;
    }

    void reset() {
        integral   = 0.0f;
        last_error = 0.0f;
        last_pwm   = 0;
    }
};

static PIDController g_pid_l;
static PIDController g_pid_r;

// ============================================================================
// micro-ROS OBJECTS
// ============================================================================

static rcl_publisher_t    g_enc_pub;
static rcl_subscription_t g_cmd_sub;
static std_msgs__msg__Int64MultiArray g_enc_msg;
static geometry_msgs__msg__Twist      g_cmd_msg;
static rclc_executor_t g_executor;
static rclc_support_t  g_support;
static rcl_allocator_t g_allocator;
static rcl_node_t      g_node;
static int64_t g_enc_data[2] = {0, 0};

// ============================================================================
// ENCODER TASK (Core 0)
// Reads both AS5600 encoders at 40 Hz.
// Wire1 may block up to 1 s on hang; Core 1 is unaffected.
// ============================================================================

static bool readAS5600(TwoWire &wire, uint16_t &prev, int32_t &steps,
                       int8_t sign, bool &init_done) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(AS5600_REG);
    if (wire.endTransmission(false) != 0) return false;
    if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;

    uint16_t hi    = wire.read();
    uint16_t lo    = wire.read();
    uint16_t angle = ((hi & 0x0F) << 8) | lo;  // 12-bit value

    if (!init_done) {
        prev      = angle;
        init_done = true;
        return true;
    }

    int16_t delta = (int16_t)(angle - prev);
    if (delta >  2048) delta -= 4096;   // wrap forward
    if (delta < -2048) delta += 4096;   // wrap backward
    steps += (int32_t)(sign * delta);
    prev = angle;
    return true;
}

static void encoderTask(void *pvp) {
    // I2C initialisation lives here — both buses owned by this task only.
    Wire.begin(L_ENC_SDA, L_ENC_SCL);
    Wire.setClock(400000);

    Wire1.begin(R_ENC_SDA, R_ENC_SCL);
    Wire1.setClock(400000);
    Wire1.setTimeout(1000);   // ESP32 bug: < 1000 ms has no effect

    int32_t  l_steps = 0, r_steps = 0;
    uint16_t l_prev  = 0, r_prev  = 0;
    bool     l_init  = false, r_init = false;
    uint32_t last_ms = millis();

    for (;;) {
        uint32_t now_ms = millis();
        float dt = (now_ms - last_ms) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        last_ms = now_ms;

        // --- Left encoder (Wire, expected fast) ---
        int32_t prev_l  = l_steps;
        bool    left_ok = readAS5600(Wire, l_prev, l_steps, L_ENC_SIGN, l_init);

        // --- Right encoder (Wire1, may hang) ---
        uint32_t t0      = millis();
        int32_t  prev_r  = r_steps;
        bool     right_ok = readAS5600(Wire1, r_prev, r_steps, R_ENC_SIGN, r_init);
        bool     hung    = (millis() - t0) > I2C_HANG_THRESH;

        if (!right_ok || hung) {
            // Bus recovery: reset Wire1
            Wire1.end();
            vTaskDelay(pdMS_TO_TICKS(10));
            Wire1.begin(R_ENC_SDA, R_ENC_SCL);
            Wire1.setClock(400000);
            Wire1.setTimeout(1000);
            r_init = false;

            if (xSemaphoreTake(g_enc_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_enc.fresh = false;
                xSemaphoreGive(g_enc_mutex);
            }
        } else if (left_ok) {
            float v_l = (l_steps - prev_l) * METERS_STEP / dt;
            float v_r = (r_steps - prev_r) * METERS_STEP / dt;

            if (xSemaphoreTake(g_enc_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_enc.left_steps  = l_steps;
                g_enc.right_steps = r_steps;
                g_enc.left_vel    = v_l;
                g_enc.right_vel   = v_r;
                g_enc.fresh       = true;
                g_enc.last_ms     = now_ms;
                xSemaphoreGive(g_enc_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(25));   // 40 Hz
    }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

static void motorsStop() {
    ledcWrite(L_PWM_CHAN, 0);
    ledcWrite(R_PWM_CHAN, 0);
}

// signed_pwm: positive = forward, negative = backward
static void setMotor(int chan, int dir_pin, int fwd_level, int signed_pwm) {
    int mag = abs(signed_pwm);
    if (mag > 0 && mag < MIN_PWM) mag = MIN_PWM;  // enforce dead-zone
    if (mag > MAX_PWM)            mag = MAX_PWM;
    bool fwd = (signed_pwm >= 0);
    digitalWrite(dir_pin, fwd ? fwd_level : !fwd_level);
    ledcWrite(chan, (uint32_t)mag);
}

// ============================================================================
// KINEMATICS
// ============================================================================

static void cmdVelToWheels(float lin, float ang, float &v_l, float &v_r) {
    v_l = lin - ang * WHEEL_BASE / 2.0f;
    v_r = lin + ang * WHEEL_BASE / 2.0f;
    v_l = constrain(v_l, -MAX_VEL, MAX_VEL);
    v_r = constrain(v_r, -MAX_VEL, MAX_VEL);
}

// ============================================================================
// micro-ROS CALLBACK
// ============================================================================

static void cmdVelCb(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msg_in;
    g_cmd.linear  = (float)msg->linear.x;
    g_cmd.angular = (float)msg->angular.z;
    g_cmd.last_ms = millis();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Motors
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    ledcSetup(L_PWM_CHAN, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(R_PWM_CHAN, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(L_PWM_PIN, L_PWM_CHAN);
    ledcAttachPin(R_PWM_PIN, R_PWM_CHAN);
    motorsStop();

    // Mutex for shared encoder state
    g_enc_mutex = xSemaphoreCreateMutex();

    // Encoder task on Core 0, priority 2 (> Arduino loop priority 1)
    xTaskCreatePinnedToCore(encoderTask, "encoders", 4096, NULL, 2, NULL, 0);

    // micro-ROS transport (921600 via arduino_transport_open override)
    set_microros_transports();
    delay(2000);

    // Wait for micro-ROS agent
    g_allocator = rcl_get_default_allocator();
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
    }
    digitalWrite(LED_PIN, HIGH);

    // Init micro-ROS
    rclc_support_init(&g_support, 0, NULL, &g_allocator);
    rmw_uros_sync_session(1000);   // synchronise wall clock
    rclc_node_init_default(&g_node, "esp32_chassis", "", &g_support);

    // Publisher: /wheel_encoders — BEST_EFFORT (sensor data, drop is acceptable)
    rclc_publisher_init_best_effort(
        &g_enc_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
        "/wheel_encoders");

    // Subscriber: /cmd_vel — RELIABLE (actuator command, must not drop)
    rclc_subscription_init_default(
        &g_cmd_sub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // Wire up encoder message memory (static array, no heap alloc)
    g_enc_msg.data.data     = g_enc_data;
    g_enc_msg.data.size     = 2;
    g_enc_msg.data.capacity = 2;

    // Executor: 1 handle (cmd_vel subscriber)
    rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator);
    rclc_executor_add_subscription(
        &g_executor, &g_cmd_sub, &g_cmd_msg, cmdVelCb, ON_NEW_DATA);

    // Grace period so watchdog does not fire before first cmd_vel
    g_cmd.last_ms = millis();
}

// ============================================================================
// LOOP (Core 1) — micro-ROS spin + PID + publish
// ============================================================================

static uint32_t last_pid_ms  = 0;
static uint32_t last_pub_ms  = 0;
static uint32_t last_sync_ms = 0;

void loop() {
    uint32_t now = millis();

    // Wall-clock drift compensation every 60 s
    if (now - last_sync_ms > 60000UL) {
        rmw_uros_sync_session(100);
        last_sync_ms = now;
    }

    // Process incoming micro-ROS messages (cmd_vel callback fires here)
    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(10));

    // Safety flags
    bool cmd_timedout = (now - g_cmd.last_ms) > CMD_TIMEOUT_MS;

    // Snapshot encoder state (quick mutex grab)
    EncoderShared enc;
    bool enc_fresh = false;
    if (xSemaphoreTake(g_enc_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        enc       = g_enc;
        enc_fresh = g_enc.fresh && (now - g_enc.last_ms < ENC_STALE_MS);
        xSemaphoreGive(g_enc_mutex);
    }

    // ---- PID at 20 Hz -------------------------------------------------------
    if (now - last_pid_ms >= PID_INTERVAL_MS) {
        float dt     = (now - last_pid_ms) / 1000.0f;
        last_pid_ms  = now;

        if (cmd_timedout || !enc_fresh) {
            // Safe stop: cut power and clear PID state (removes integral windup)
            motorsStop();
            g_pid_l.reset();
            g_pid_r.reset();
        } else {
            float v_l_target, v_r_target;
            cmdVelToWheels(g_cmd.linear, g_cmd.angular, v_l_target, v_r_target);

            int pwm_l = g_pid_l.update(v_l_target, enc.left_vel,  dt);
            int pwm_r = g_pid_r.update(v_r_target, enc.right_vel, dt);

            setMotor(L_PWM_CHAN, L_DIR_PIN, L_DIR_FWD, pwm_l);
            setMotor(R_PWM_CHAN, R_DIR_PIN, R_DIR_FWD, pwm_r);
        }
    }

    // ---- Publish /wheel_encoders at 20 Hz (only when encoder is fresh) ------
    if (now - last_pub_ms >= 50UL) {
        last_pub_ms = now;
        if (enc_fresh) {
            g_enc_data[0] = (int64_t)enc.left_steps;
            g_enc_data[1] = (int64_t)enc.right_steps;
            rcl_publish(&g_enc_pub, &g_enc_msg, NULL);
        }
    }

    // ---- LED status ---------------------------------------------------------
    // Fast blink: cmd_vel timeout  |  Slow blink: encoder stale  |  Solid: OK
    if (cmd_timedout) {
        digitalWrite(LED_PIN, (now % 200) < 100);
    } else if (!enc_fresh) {
        digitalWrite(LED_PIN, (now % 1000) < 500);
    } else {
        digitalWrite(LED_PIN, HIGH);
    }

    delay(1);
}
