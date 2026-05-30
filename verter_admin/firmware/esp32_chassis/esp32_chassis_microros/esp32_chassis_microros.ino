/**
 * ESP32 Chassis Controller v2 — micro-ROS
 *
 * L0 safety-critical actuation layer (per SDHR rulebook §2.1).
 * Owns: motor output, cmd_vel watchdog, bounded velocity.
 *
 * Architecture
 * ------------
 * Core 0: encL_task  (prio 3, 100 Hz) — Wire  / left  AS5600
 *         encR_task  (prio 3, 100 Hz) — Wire1 / right AS5600 (isolated — its
 *                                       hang does NOT block the left encoder)
 * Core 1: controlTask (prio 4,  50 Hz, vTaskDelayUntil) — FF+PI + safety SM
 *         microrosTask(prio 2)       — executor spin + all publishing
 *
 * Shared state protected by portMUX_TYPE spinlocks (short critical sections).
 * Zero dynamic allocation after setup. Static message buffers only. (§7)
 *
 * State machine (§4):
 *   INIT → WAIT_AGENT → READY → ACTIVE ⇄ SAFE_STOP
 *                                   ↓
 *                                 FAULT (manual reboot only)
 *
 * Safety invariants (§3.3):
 *   I1: cmd_vel timeout (500 ms)     → SAFE_STOP
 *   I2: encoder stale  (L or R, 1 s) → SAFE_STOP
 *   I3: |v_target| ≤ MAX_VEL
 *   I4: |pwm|      ≤ MAX_PWM
 *   I5: startup default = motors OFF until READY + fresh cmd_vel + fresh encoders
 *
 * Controller: Feed-Forward + PI (no D, no ramp, no MIN_PWM dead-zone).
 *   FF linearises PWM→velocity: pwm_ff = sign(v)*k_s + k_v*v
 *   PI trims residual velocity error.
 *   Stick-slip avoidance: FF supplies stiction PWM at first tick, no ramp-up lag.
 *
 * Interfaces (§5)
 * ---------------
 * Sub  /cmd_vel                geometry_msgs/Twist           RELIABLE
 * Pub  /wheel_encoders         std_msgs/Int64MultiArray[2]   BEST_EFFORT  50 Hz  (only when fresh)
 * Pub  /chassis/state          std_msgs/UInt8               RELIABLE      2 Hz  heartbeat
 * Pub  /chassis/pwm            std_msgs/Int16MultiArray[2]  BEST_EFFORT  20 Hz  debug
 * Pub  /chassis/pid_debug      std_msgs/Float32MultiArray[4] BEST_EFFORT 20 Hz  integL,integR,errL,errR
 * Pub  /chassis/hang_counters  std_msgs/UInt32MultiArray[2] RELIABLE      2 Hz
 * Pub  /chassis/cmd_watchdog   std_msgs/Bool                RELIABLE      2 Hz
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/u_int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <Wire.h>

// ============================================================================
// HARDWARE / ROBOT CONSTANTS
// ============================================================================

// Motors (Cytron MD10C: PWM + DIR)
#define L_PWM_PIN   19
#define L_DIR_PIN   27
#define R_PWM_PIN   18
#define R_DIR_PIN   25
#define L_DIR_FWD   LOW    // LOW  = forward for left  motor
#define R_DIR_FWD   HIGH   // HIGH = forward for right motor

#define L_PWM_CHAN   0
#define R_PWM_CHAN   1
#define PWM_FREQ_HZ  20000
#define PWM_BITS     8
#define MAX_PWM      200

// Encoders (AS5600, 12-bit absolute magnetic)
#define L_ENC_SDA    21
#define L_ENC_SCL    22
#define R_ENC_SDA    32
#define R_ENC_SCL    4
#define AS5600_ADDR  0x36
// ANGLE register (0x0E) — built-in slow-filter + hysteresis.
// RAW_ANGLE (0x0C) is unfiltered and has ±2-5 LSB noise at rest that
// accumulates as phantom step drift, especially on Wire1 (shares GPIO 4
// with Tegra USB, picks up electrical noise).
#define AS5600_REG   0x0E
#define ENC_RES      4096
#define L_ENC_SIGN   (-1)   // left encoder counts DOWN when forward
#define R_ENC_SIGN   (+1)   // right encoder counts UP   when forward

// Sanity filter: reject read if |delta| per tick exceeds physical max.
// At MAX_VEL=0.5 m/s → ~142 steps/10ms tick. 220 gives ~1.5× margin.
#define MAX_STEP_DELTA 220

// Kinematics (hardware_spec.md)
static constexpr float WHEEL_CIRC  = 0.576f;
static constexpr float GEAR_RATIO  = 4.0007f;
static constexpr float WHEEL_BASE  = 0.386f;
static constexpr float METERS_STEP = WHEEL_CIRC / (ENC_RES * GEAR_RATIO);
static constexpr float MAX_VEL     = 0.5f;

// Feed-Forward + PI controller
// STARTING VALUES — re-calibrate with calibration_node step-response.
//   FF_KV: PWM per m/s. 360 → at MAX_VEL=0.5 base PWM=180, leaves ~20 for PI.
//   FF_KS: static-friction PWM. Motors start turning near 25 under load, use 28.
static constexpr float FF_KV    = 360.0f;  // TODO calibrate
static constexpr float FF_KS    = 28.0f;   // TODO calibrate
static constexpr float PID_KP   = 40.0f;
static constexpr float PID_KI   = 20.0f;
static constexpr float PID_IMAX = 50.0f;   // anti-windup clamp

// Timing / safety (§3.2)
static constexpr uint32_t CTRL_PERIOD_MS = 20;    //  50 Hz PID
static constexpr uint32_t ENC_PERIOD_MS  = 10;    // 100 Hz encoder read
static constexpr uint32_t CMD_TIMEOUT_MS = 500;   // matches MotionTimeouts contract
static constexpr uint32_t ENC_TIMEOUT_MS = 1500;  // Wire1 hangs up to ~1 s on Jetson-cp210x
static constexpr uint32_t I2C_HANG_US    = 60000; // Wire read >60 ms → bus reset
static constexpr uint32_t FAULT_HANG_MS  = 10000; // both wheels dead this long → FAULT

#define LED_PIN 2

// micro-ROS serial baud override
extern "C" bool arduino_transport_open(struct uxrCustomTransport *) {
    Serial.begin(921600);
    return true;
}

// ============================================================================
// STATE MACHINE
// ============================================================================

enum SystemState : uint8_t {
    STATE_INIT       = 0,
    STATE_WAIT_AGENT = 1,
    STATE_READY      = 2,  // agent connected, waiting for cmd_vel + encoders
    STATE_ACTIVE     = 3,  // driving motors
    STATE_SAFE_STOP  = 4,  // transient fault (cmd/enc timeout); auto-recovers
    STATE_FAULT      = 5,  // persistent hardware fault; requires reboot
};

// ============================================================================
// SHARED STATE (cross-core)
// ============================================================================

struct EncoderShared {
    int32_t  steps;           // cumulative signed step count
    uint32_t last_update_ms;  // millis() of last successful read
    uint32_t hang_count;      // cumulative I2C hang events
    bool     initialized;
};
static EncoderShared g_encL = {};
static EncoderShared g_encR = {};
static portMUX_TYPE  g_encL_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE  g_encR_mux = portMUX_INITIALIZER_UNLOCKED;

struct CmdShared {
    float    linear;
    float    angular;
    uint32_t last_update_ms;
};
static CmdShared    g_cmd = {};
static portMUX_TYPE g_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

struct ControlSnapshot {
    int16_t pwm_l, pwm_r;
    float   integ_l, integ_r;
    float   err_l,   err_r;
    uint8_t state;
    bool    cmd_fresh;
};
static ControlSnapshot g_snap = {};
static portMUX_TYPE    g_snap_mux = portMUX_INITIALIZER_UNLOCKED;

// Written by microrosTask (during bringup) and controlTask. uint8_t write is
// atomic on Xtensa so no lock needed for the state itself — g_snap protects
// the full snapshot.
static volatile SystemState g_state = STATE_INIT;

// ============================================================================
// ENCODER TASK — one instance per I2C bus, fully isolated
// ============================================================================

struct EncoderTaskArgs {
    TwoWire*       wire;
    int            sda_pin;
    int            scl_pin;
    int8_t         sign;
    EncoderShared* shared;
    portMUX_TYPE*  mux;
    const char*    name;
};

static bool readAS5600(TwoWire &wire, uint16_t &out_angle) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(AS5600_REG);
    if (wire.endTransmission(false) != 0) return false;
    if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;
    uint16_t hi = wire.read();
    uint16_t lo = wire.read();
    out_angle = ((hi & 0x0F) << 8) | lo;
    return true;
}

static void encoderTask(void *pvp) {
    EncoderTaskArgs *args = (EncoderTaskArgs *)pvp;

    args->wire->begin(args->sda_pin, args->scl_pin);
    args->wire->setClock(400000);
    args->wire->setTimeout(1000);   // arduino-esp32 v2.x bug: <1000 has no effect

    // Give AS5600 time to power-up before first transaction.
    // Without this, Wire1's first read can hang on some boards, and the
    // recovery loop never escapes.
    vTaskDelay(pdMS_TO_TICKS(50));

    uint16_t prev_angle  = 0;
    int32_t  steps_local = 0;
    bool     inited      = false;
    uint32_t hangs       = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENC_PERIOD_MS));

        uint32_t t0_us = micros();
        uint16_t angle;
        bool     ok     = readAS5600(*args->wire, angle);
        uint32_t dt_us  = micros() - t0_us;
        bool     hung   = dt_us > I2C_HANG_US;

        if (!ok || hung) {
            hangs++;
            // Bus recovery (known Jetson-cp210x / arduino-esp32 Wire1 issue)
            args->wire->end();
            vTaskDelay(pdMS_TO_TICKS(10));
            args->wire->begin(args->sda_pin, args->scl_pin);
            args->wire->setClock(400000);
            args->wire->setTimeout(1000);
            inited = false;

            // Publish only hang counter; steps/last_update_ms left as-is so
            // controlTask detects staleness via ENC_TIMEOUT_MS.
            portENTER_CRITICAL(args->mux);
            args->shared->hang_count = hangs;
            portEXIT_CRITICAL(args->mux);
            continue;
        }

        if (!inited) {
            prev_angle = angle;
            inited     = true;
            uint32_t now = millis();
            portENTER_CRITICAL(args->mux);
            args->shared->initialized    = true;
            args->shared->last_update_ms = now;
            portEXIT_CRITICAL(args->mux);
            continue;
        }

        int16_t delta = (int16_t)(angle - prev_angle);
        if (delta >  2048) delta -= 4096;
        if (delta < -2048) delta += 4096;

        // Reject glitched reads: physical wheel can't move >MAX_STEP_DELTA
        // per ENC_PERIOD_MS tick. Re-baseline prev_angle so we don't
        // compound the rejection into a huge delta next tick.
        if (abs(delta) > MAX_STEP_DELTA) {
            prev_angle = angle;
            continue;
        }

        prev_angle   = angle;
        steps_local += (int32_t)(args->sign * delta);

        uint32_t now = millis();
        portENTER_CRITICAL(args->mux);
        args->shared->steps          = steps_local;
        args->shared->last_update_ms = now;
        args->shared->hang_count     = hangs;
        portEXIT_CRITICAL(args->mux);
    }
}

// ============================================================================
// MOTOR OUTPUT
// ============================================================================

static inline void motorsStop() {
    ledcWrite(L_PWM_CHAN, 0);
    ledcWrite(R_PWM_CHAN, 0);
}

static inline void setMotor(int pwm_chan, int dir_pin, int fwd_level, int16_t signed_pwm) {
    int16_t mag = abs(signed_pwm);
    if (mag > MAX_PWM) mag = MAX_PWM;   // I4
    bool fwd = (signed_pwm >= 0);
    digitalWrite(dir_pin, fwd ? fwd_level : !fwd_level);
    ledcWrite(pwm_chan, (uint32_t)mag);
}

// ============================================================================
// CONTROLLER — Feed-Forward + PI per wheel
// Implemented as a class so Arduino IDE's ctags does not auto-generate
// forward declarations with user-defined types (causes build failure).
// ============================================================================

class PIController {
public:
    float integral = 0.0f;
    float last_err = 0.0f;

    int16_t update(float target, float actual, float dt) {
        // Zero command → flush and stop cleanly (prevents integral lingering)
        if (fabsf(target) < 1e-3f) {
            integral = 0.0f;
            last_err = 0.0f;
            return 0;
        }

        float ff  = (target > 0 ? FF_KS : -FF_KS) + FF_KV * target;
        float err = target - actual;

        integral += err * dt;
        if (integral >  PID_IMAX) integral =  PID_IMAX;
        if (integral < -PID_IMAX) integral = -PID_IMAX;
        last_err = err;

        float fb  = PID_KP * err + PID_KI * integral;
        float raw = ff + fb;
        if (raw >  MAX_PWM) raw =  MAX_PWM;
        if (raw < -MAX_PWM) raw = -MAX_PWM;
        return (int16_t)raw;
    }

    void reset() {
        integral = 0.0f;
        last_err = 0.0f;
    }
};

static PIController g_piL;
static PIController g_piR;

static inline void cmdVelToWheels(float lin, float ang, float &v_l, float &v_r) {
    if (lin >  MAX_VEL) lin =  MAX_VEL;   // I3
    if (lin < -MAX_VEL) lin = -MAX_VEL;
    v_l = lin - ang * WHEEL_BASE / 2.0f;
    v_r = lin + ang * WHEEL_BASE / 2.0f;
    if (v_l >  MAX_VEL) v_l =  MAX_VEL;
    if (v_l < -MAX_VEL) v_l = -MAX_VEL;
    if (v_r >  MAX_VEL) v_r =  MAX_VEL;
    if (v_r < -MAX_VEL) v_r = -MAX_VEL;
}

// ============================================================================
// CONTROL TASK (Core 1) — deterministic 50 Hz safety + PID
// ============================================================================

static void controlTask(void *pvp) {
    // Velocity estimation (local — not shared)
    int32_t  prev_L_steps = 0, prev_R_steps = 0;
    uint32_t prev_ctrl_ms = 0;
    float    vL_filt      = 0.0f, vR_filt = 0.0f;
    constexpr float VEL_EMA_ALPHA = 0.4f;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));

        uint32_t now = millis();

        // --- Snapshot shared state ------------------------------------------
        EncoderShared encL, encR;
        portENTER_CRITICAL(&g_encL_mux); encL = g_encL; portEXIT_CRITICAL(&g_encL_mux);
        portENTER_CRITICAL(&g_encR_mux); encR = g_encR; portEXIT_CRITICAL(&g_encR_mux);

        CmdShared cmd;
        portENTER_CRITICAL(&g_cmd_mux); cmd = g_cmd; portEXIT_CRITICAL(&g_cmd_mux);

        // --- Freshness checks (I1, I2) --------------------------------------
        bool cmd_fresh  = (cmd.last_update_ms > 0) &&
                          ((now - cmd.last_update_ms) < CMD_TIMEOUT_MS);
        bool encL_fresh = encL.initialized &&
                          ((now - encL.last_update_ms) < ENC_TIMEOUT_MS);
        bool encR_fresh = encR.initialized &&
                          ((now - encR.last_update_ms) < ENC_TIMEOUT_MS);
        bool enc_fresh  = encL_fresh && encR_fresh;

        // --- FAULT transition: both encoders dead for FAULT_HANG_MS ---------
        if (g_state != STATE_FAULT && g_state != STATE_INIT && g_state != STATE_WAIT_AGENT) {
            bool encL_dead = !encL_fresh && encL.hang_count > 0 &&
                             (now - encL.last_update_ms) > FAULT_HANG_MS;
            bool encR_dead = !encR_fresh && encR.hang_count > 0 &&
                             (now - encR.last_update_ms) > FAULT_HANG_MS;
            if (encL_dead && encR_dead) g_state = STATE_FAULT;
        }

        // --- Velocity estimation (EMA on per-tick Δsteps) -------------------
        float dt_s = (prev_ctrl_ms > 0) ? (now - prev_ctrl_ms) / 1000.0f
                                        : (CTRL_PERIOD_MS / 1000.0f);
        if (dt_s < 0.001f) dt_s = CTRL_PERIOD_MS / 1000.0f;

        float vL_raw = (encL.steps - prev_L_steps) * METERS_STEP / dt_s;
        float vR_raw = (encR.steps - prev_R_steps) * METERS_STEP / dt_s;
        prev_L_steps = encL.steps;
        prev_R_steps = encR.steps;
        prev_ctrl_ms = now;

        vL_filt += VEL_EMA_ALPHA * (vL_raw - vL_filt);
        vR_filt += VEL_EMA_ALPHA * (vR_raw - vR_filt);

        // --- State machine --------------------------------------------------
        int16_t pwm_l = 0, pwm_r = 0;

        switch (g_state) {
        case STATE_INIT:
        case STATE_WAIT_AGENT:
            motorsStop();
            break;

        case STATE_READY:
            motorsStop();
            g_piL.reset();
            g_piR.reset();
            vL_filt = vR_filt = 0.0f;
            if (cmd_fresh && enc_fresh) g_state = STATE_ACTIVE;
            break;

        case STATE_ACTIVE: {
            if (!cmd_fresh || !enc_fresh) {
                g_state = STATE_SAFE_STOP;
                motorsStop();
                g_piL.reset();
                g_piR.reset();
                break;
            }
            float v_l_tgt, v_r_tgt;
            cmdVelToWheels(cmd.linear, cmd.angular, v_l_tgt, v_r_tgt);
            pwm_l = g_piL.update(v_l_tgt, vL_filt, dt_s);
            pwm_r = g_piR.update(v_r_tgt, vR_filt, dt_s);
            setMotor(L_PWM_CHAN, L_DIR_PIN, L_DIR_FWD, pwm_l);
            setMotor(R_PWM_CHAN, R_DIR_PIN, R_DIR_FWD, pwm_r);
            break;
        }

        case STATE_SAFE_STOP:
            motorsStop();
            if (cmd_fresh && enc_fresh) {
                g_piL.reset();
                g_piR.reset();
                vL_filt = vR_filt = 0.0f;
                g_state = STATE_ACTIVE;
            }
            break;

        case STATE_FAULT:
            motorsStop();
            // no auto-recovery
            break;
        }

        // --- Publish snapshot for microrosTask ------------------------------
        portENTER_CRITICAL(&g_snap_mux);
        g_snap.pwm_l     = pwm_l;
        g_snap.pwm_r     = pwm_r;
        g_snap.integ_l   = g_piL.integral;
        g_snap.integ_r   = g_piR.integral;
        g_snap.err_l     = g_piL.last_err;
        g_snap.err_r     = g_piR.last_err;
        g_snap.state     = (uint8_t)g_state;
        g_snap.cmd_fresh = cmd_fresh;
        portEXIT_CRITICAL(&g_snap_mux);
    }
}

// ============================================================================
// micro-ROS OBJECTS
// ============================================================================

static rcl_node_t       g_node;
static rclc_support_t   g_support;
static rcl_allocator_t  g_allocator;
static rclc_executor_t  g_executor;

static rcl_subscription_t g_sub_cmd;
static rcl_publisher_t    g_pub_enc;
static rcl_publisher_t    g_pub_state;
static rcl_publisher_t    g_pub_pwm;
static rcl_publisher_t    g_pub_piddbg;
static rcl_publisher_t    g_pub_hangs;
static rcl_publisher_t    g_pub_wd;

static geometry_msgs__msg__Twist        g_msg_cmd;
static std_msgs__msg__Int64MultiArray   g_msg_enc;
static std_msgs__msg__UInt8             g_msg_state;
static std_msgs__msg__Int16MultiArray   g_msg_pwm;
static std_msgs__msg__Float32MultiArray g_msg_piddbg;
static std_msgs__msg__UInt32MultiArray  g_msg_hangs;
static std_msgs__msg__Bool              g_msg_wd;

static int64_t  g_enc_data[2]    = {0, 0};
static int16_t  g_pwm_data[2]    = {0, 0};
static float    g_piddbg_data[4] = {0, 0, 0, 0};
static uint32_t g_hangs_data[2]  = {0, 0};

static void cmdVelCb(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    uint32_t now = millis();
    portENTER_CRITICAL(&g_cmd_mux);
    g_cmd.linear         = (float)msg->linear.x;
    g_cmd.angular        = (float)msg->angular.z;
    g_cmd.last_update_ms = now;
    portEXIT_CRITICAL(&g_cmd_mux);
}

static void microrosTask(void *pvp) {
    g_state = STATE_WAIT_AGENT;
    g_allocator = rcl_get_default_allocator();

    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    digitalWrite(LED_PIN, HIGH);

    rclc_support_init(&g_support, 0, NULL, &g_allocator);
    rmw_uros_sync_session(1000);
    rclc_node_init_default(&g_node, "esp32_chassis", "", &g_support);

    rclc_subscription_init_default(&g_sub_cmd, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

    rclc_publisher_init_best_effort(&g_pub_enc, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray), "/wheel_encoders");
    rclc_publisher_init_default(&g_pub_state, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/chassis/state");
    rclc_publisher_init_best_effort(&g_pub_pwm, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/chassis/pwm");
    rclc_publisher_init_best_effort(&g_pub_piddbg, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/chassis/pid_debug");
    rclc_publisher_init_default(&g_pub_hangs, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32MultiArray), "/chassis/hang_counters");
    rclc_publisher_init_default(&g_pub_wd, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/chassis/cmd_watchdog");

    // Static buffers
    g_msg_enc.data.data        = g_enc_data;
    g_msg_enc.data.size        = 2;   g_msg_enc.data.capacity    = 2;
    g_msg_pwm.data.data        = g_pwm_data;
    g_msg_pwm.data.size        = 2;   g_msg_pwm.data.capacity    = 2;
    g_msg_piddbg.data.data     = g_piddbg_data;
    g_msg_piddbg.data.size     = 4;   g_msg_piddbg.data.capacity = 4;
    g_msg_hangs.data.data      = g_hangs_data;
    g_msg_hangs.data.size      = 2;   g_msg_hangs.data.capacity  = 2;

    rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator);
    rclc_executor_add_subscription(&g_executor, &g_sub_cmd, &g_msg_cmd, cmdVelCb, ON_NEW_DATA);

    g_state = STATE_READY;

    uint32_t last_enc_ms    = 0;
    uint32_t last_pwm_ms    = 0;
    uint32_t last_piddbg_ms = 0;
    uint32_t last_state_ms  = 0;
    uint32_t last_hangs_ms  = 0;
    uint32_t last_wd_ms     = 0;
    uint32_t last_sync_ms   = millis();

    for (;;) {
        rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));
        uint32_t now = millis();

        if (now - last_sync_ms > 60000UL) {
            rmw_uros_sync_session(100);
            last_sync_ms = now;
        }

        // Snapshot consumer state
        ControlSnapshot snap;
        portENTER_CRITICAL(&g_snap_mux); snap = g_snap; portEXIT_CRITICAL(&g_snap_mux);
        EncoderShared encL, encR;
        portENTER_CRITICAL(&g_encL_mux); encL = g_encL; portEXIT_CRITICAL(&g_encL_mux);
        portENTER_CRITICAL(&g_encR_mux); encR = g_encR; portEXIT_CRITICAL(&g_encR_mux);

        bool enc_fresh = encL.initialized && encR.initialized &&
                         ((now - encL.last_update_ms) < ENC_TIMEOUT_MS) &&
                         ((now - encR.last_update_ms) < ENC_TIMEOUT_MS);

        // /wheel_encoders — 50 Hz, only when both wheels fresh
        if ((now - last_enc_ms) >= 20UL) {
            last_enc_ms = now;
            if (enc_fresh) {
                g_enc_data[0] = (int64_t)encL.steps;
                g_enc_data[1] = (int64_t)encR.steps;
                rcl_publish(&g_pub_enc, &g_msg_enc, NULL);
            }
        }

        // /chassis/pwm — 20 Hz
        if ((now - last_pwm_ms) >= 50UL) {
            last_pwm_ms = now;
            g_pwm_data[0] = snap.pwm_l;
            g_pwm_data[1] = snap.pwm_r;
            rcl_publish(&g_pub_pwm, &g_msg_pwm, NULL);
        }

        // /chassis/pid_debug — 20 Hz
        if ((now - last_piddbg_ms) >= 50UL) {
            last_piddbg_ms = now;
            g_piddbg_data[0] = snap.integ_l;
            g_piddbg_data[1] = snap.integ_r;
            g_piddbg_data[2] = snap.err_l;
            g_piddbg_data[3] = snap.err_r;
            rcl_publish(&g_pub_piddbg, &g_msg_piddbg, NULL);
        }

        // /chassis/state — 2 Hz heartbeat
        if ((now - last_state_ms) >= 500UL) {
            last_state_ms = now;
            g_msg_state.data = snap.state;
            rcl_publish(&g_pub_state, &g_msg_state, NULL);
        }

        // /chassis/hang_counters — 2 Hz
        if ((now - last_hangs_ms) >= 500UL) {
            last_hangs_ms = now;
            g_hangs_data[0] = encL.hang_count;
            g_hangs_data[1] = encR.hang_count;
            rcl_publish(&g_pub_hangs, &g_msg_hangs, NULL);
        }

        // /chassis/cmd_watchdog — 2 Hz
        if ((now - last_wd_ms) >= 500UL) {
            last_wd_ms = now;
            g_msg_wd.data = snap.cmd_fresh;
            rcl_publish(&g_pub_wd, &g_msg_wd, NULL);
        }

        // LED: ACTIVE solid, SAFE_STOP slow blink, FAULT fast blink, else breathing
        switch ((SystemState)snap.state) {
        case STATE_ACTIVE:    digitalWrite(LED_PIN, HIGH);                           break;
        case STATE_SAFE_STOP: digitalWrite(LED_PIN, (now %  400) <  200);            break;
        case STATE_FAULT:     digitalWrite(LED_PIN, (now %  100) <   50);            break;
        default:              digitalWrite(LED_PIN, (now % 1000) <  500);            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// SETUP
// ============================================================================

static EncoderTaskArgs g_encL_args =
    {&Wire,  L_ENC_SDA, L_ENC_SCL, L_ENC_SIGN, &g_encL, &g_encL_mux, "encL"};
static EncoderTaskArgs g_encR_args =
    {&Wire1, R_ENC_SDA, R_ENC_SCL, R_ENC_SIGN, &g_encR, &g_encR_mux, "encR"};

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Motor outputs in SAFE state before anything else (I5)
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    ledcSetup(L_PWM_CHAN, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(R_PWM_CHAN, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(L_PWM_PIN, L_PWM_CHAN);
    ledcAttachPin(R_PWM_PIN, R_PWM_CHAN);
    motorsStop();

    set_microros_transports();
    delay(2000);

    // Encoder tasks — isolated on Core 0, one per I2C bus
    xTaskCreatePinnedToCore(encoderTask, "encL", 4096, &g_encL_args, 3, NULL, 0);
    xTaskCreatePinnedToCore(encoderTask, "encR", 4096, &g_encR_args, 3, NULL, 0);

    // Control task — deterministic 50 Hz, higher prio than micro-ROS
    xTaskCreatePinnedToCore(controlTask,  "ctrl", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(microrosTask, "uros", 8192, NULL, 2, NULL, 1);
}

void loop() {
    // All work happens in FreeRTOS tasks. Keep Arduino loopTask alive but idle.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
