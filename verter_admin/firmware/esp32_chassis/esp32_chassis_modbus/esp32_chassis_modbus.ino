/**
 * ESP32 Chassis Controller — micro-ROS + ZLAC8015D over Modbus RTU
 *
 * Replaces the previous AS5600 + Cytron MD10C firmware. ESP32 is the L1
 * mediator between Jetson (ROS 2) and the ZLAC8015D dual-channel servo driver:
 *
 *   Jetson ── USB CDC ── ESP32 ── UART2/TTL-RS485 ── ZLAC8015D ── motors
 *           (micro-ROS)         (Modbus RTU @ 115200)
 *
 * ESP32 owns the command path safety (cmd_vel watchdog, state machine,
 * kinematics, sign mapping). ZLAC owns the closed-loop motor control and a
 * hardware comm-loss watchdog. There is no physical E-stop input to firmware
 * — the hardware E-stop sits in the 12V power line and cold-boots the MCU
 * when released.
 *
 * Architecture
 * ------------
 * Core 1: controlTask  (prio 3, 50 Hz, vTaskDelayUntil)
 *           snapshot cmd_vel → kinematics → Modbus write target RPM →
 *           Modbus read feedback → state machine → publish snapshot
 *         microrosTask (prio 2)
 *           executor spin + publish to ROS
 *
 * State machine
 *   INIT → WAIT_AGENT → READY → ACTIVE ⇄ SAFE_STOP
 *                                   ↓
 *                                 FAULT (auto-retry clear every 2 s)
 *
 * Topic contracts (unchanged from prior firmware so odometry_node etc. work)
 *   Sub  /cmd_vel              geometry_msgs/Twist            RELIABLE
 *   Pub  /wheel_encoders       std_msgs/Int64MultiArray[2]    BEST_EFFORT  50 Hz
 *   Pub  /chassis/state        std_msgs/UInt8                 RELIABLE      2 Hz
 *   Pub  /chassis/fault        std_msgs/UInt32                RELIABLE      2 Hz
 *   Pub  /chassis/cmd_watchdog std_msgs/Bool                  RELIABLE      2 Hz
 *   Pub  /chassis/wheel_rpm    std_msgs/Int16MultiArray[2]    BEST_EFFORT  20 Hz
 *
 * Wiring (per docs/weels/verter_zlac_8015_d_hardware_summary_md.md)
 *   GPIO17 (TX2) → TTL-RS485 RXD
 *   GPIO16 (RX2) → TTL-RS485 TXD
 *   GND, 3V3     → TTL-RS485 GND, VCC
 *   USB          → Jetson (micro-ROS Agent)
 *
 * Register map: typical ZLTECH ZLAC8015D family.  Marked TODO VERIFY; if a
 * register read returns Modbus exception on bench, cross-check against the
 * manual delivered with the unit and adjust the #defines below.
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>

// ============================================================================
// WIRING
// ============================================================================

#define UART_MODBUS         Serial2
#define UART_MODBUS_BAUD    115200
// Engineer's hardware-summary said TX2=GPIO17, RX2=GPIO16, but the actual
// wiring on this robot is the opposite (confirmed by scanner on 2026-05-31:
// only TX=16 RX=17 gets a Modbus response from ZLAC).
#define UART_MODBUS_TX_PIN  16
#define UART_MODBUS_RX_PIN  17

#define LED_PIN             2

// ============================================================================
// ZLAC8015D MODBUS REGISTER MAP — TODO VERIFY against manual
// ============================================================================

#define ZLAC_SLAVE_ADDR             0x01

#define REG_FW_VERSION              0x0001  // R

#define REG_COMM_WATCHDOG_MS        0x2007  // W, ms — driver brakes on loss
#define COMM_WATCHDOG_MS            200

#define REG_OPERATION_MODE          0x200D  // W, set once at setup
#define MODE_VELOCITY               3

#define REG_CONTROL_WORD            0x200E  // W
#define CTRL_ENABLE                 0x0008
#define CTRL_DISABLE                0x0007
#define CTRL_CLEAR_FAULT            0x0006
#define CTRL_EMERGENCY_STOP         0x0005

#define REG_TARGET_RPM_M1           0x2088  // W, signed int16 RPM
#define REG_TARGET_RPM_M2           0x2089

#define REG_DRIVER_STATUS           0x20A5  // R, status bit-field
#define REG_FAULT_CODE              0x20A6  // R, 0 = no fault
#define REG_POSITION_M1_HI          0x20A7  // R, int32 = hi<<16 | lo
#define REG_POSITION_M1_LO          0x20A8
#define REG_POSITION_M2_HI          0x20A9
#define REG_POSITION_M2_LO          0x20AA
#define REG_ACTUAL_RPM_M1           0x20AB  // R, signed int16 RPM
#define REG_ACTUAL_RPM_M2           0x20AC

#define FC_READ_HOLDING             0x03
#define FC_WRITE_SINGLE             0x06
#define FC_WRITE_MULTIPLE           0x10

// ============================================================================
// ROBOT KINEMATICS / POLICY
// ============================================================================

static constexpr float WHEEL_DIAMETER   = 0.200f;   // m  (ZLLG80ASM250)
static constexpr float WHEEL_BASE       = 0.386f;   // m  TODO verify on chassis
static constexpr float GEAR_RATIO       = 1.0f;     // direct-drive
static constexpr float MAX_LINEAR_VEL   = 0.5f;     // m/s
static constexpr float MAX_ANGULAR_VEL  = 1.0f;     // rad/s
static constexpr int16_t MAX_MOTOR_RPM  = 200;      // rated for ZLLG80ASM250

// Sign calibration — TODO verify on bench (spin one motor at +20 RPM,
// confirm wheel rotates "forward" relative to robot front).
static constexpr int8_t  LEFT_SIGN      = +1;
static constexpr int8_t  RIGHT_SIGN     = -1;

// Derived: motor RPM per m/s of wheel speed
static constexpr float MOTOR_RPM_PER_MPS =
    60.0f / (3.14159265f * WHEEL_DIAMETER) * GEAR_RATIO;

// ============================================================================
// TIMING / SAFETY
// ============================================================================

static constexpr uint32_t CTRL_PERIOD_MS    = 20;    // 50 Hz
static constexpr uint32_t CMD_TIMEOUT_MS    = 500;   // cmd_vel watchdog
static constexpr uint32_t MODBUS_TIMEOUT_MS = 100;   // per-frame
static constexpr uint8_t  MODBUS_MAX_FAILS  = 5;     // consecutive → SAFE_STOP
static constexpr uint32_t FAULT_RETRY_MS    = 2000;  // auto-clear retry
static constexpr uint32_t SETUP_RETRY_MS    = 1000;  // READY → enable retry

// ============================================================================
// STATE MACHINE
// ============================================================================

enum SystemState : uint8_t {
    STATE_INIT       = 0,
    STATE_WAIT_AGENT = 1,
    STATE_READY      = 2,
    STATE_ACTIVE     = 3,
    STATE_SAFE_STOP  = 4,
    STATE_FAULT      = 5,
};

// ============================================================================
// SHARED STATE (cross-task)
// ============================================================================

struct CmdShared {
    float    linear;
    float    angular;
    uint32_t last_update_ms;
};
static CmdShared    g_cmd = {};
static portMUX_TYPE g_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

struct FeedbackShared {
    int16_t  actual_rpm_l;
    int16_t  actual_rpm_r;
    int64_t  position_l;     // accumulated across int32 wraps — never wraps
    int64_t  position_r;
    uint16_t fault_code;
    uint16_t modbus_fails;   // consecutive failures
    bool     fresh;          // any successful read since boot
};
static FeedbackShared g_fb = {};
static portMUX_TYPE   g_fb_mux = portMUX_INITIALIZER_UNLOCKED;

struct ControlSnapshot {
    int16_t target_rpm_l;
    int16_t target_rpm_r;
    uint8_t state;
    bool    cmd_fresh;
};
static ControlSnapshot g_snap = {};
static portMUX_TYPE    g_snap_mux = portMUX_INITIALIZER_UNLOCKED;

// One-shot feedback read result. Declared up here (not next to its function)
// because Arduino IDE's auto-prototype generator places forward declarations
// for ALL functions at the very top of the translation unit — if the struct
// were declared after the function, the auto-generated prototype would
// reference an unknown type and the build would fail.
struct ZLACFeedback {
    int16_t  rpm_l, rpm_r;
    int32_t  pos_l, pos_r;
    uint16_t fault;
};

// uint8 writes are atomic on Xtensa; bare volatile is enough for state.
static volatile SystemState g_state = STATE_INIT;

// micro-ROS serial baud override (must match agent side)
extern "C" bool arduino_transport_open(struct uxrCustomTransport*) {
    Serial.begin(921600);
    return true;
}

// ============================================================================
// MODBUS RTU — implementation
// ============================================================================

static uint16_t modbus_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

static void modbus_drain() {
    while (UART_MODBUS.available()) UART_MODBUS.read();
}

// Append CRC then write `len + 2` bytes. `frame` MUST have +2 bytes of headroom.
static void modbus_send(uint8_t* frame, size_t len) {
    uint16_t crc = modbus_crc16(frame, len);
    frame[len]     =  crc       & 0xFF;
    frame[len + 1] = (crc >> 8) & 0xFF;
    UART_MODBUS.write(frame, len + 2);
    UART_MODBUS.flush();
}

static bool modbus_recv(uint8_t* buf, size_t expected, uint32_t timeout_ms) {
    uint32_t deadline = millis() + timeout_ms;
    size_t   got = 0;
    while (got < expected && (int32_t)(deadline - millis()) > 0) {
        if (UART_MODBUS.available()) {
            buf[got++] = UART_MODBUS.read();
        } else {
            vTaskDelay(1);
        }
    }
    if (got != expected) return false;
    uint16_t crc_calc = modbus_crc16(buf, expected - 2);
    uint16_t crc_recv = (uint16_t)buf[expected - 2] |
                        ((uint16_t)buf[expected - 1] << 8);
    return crc_calc == crc_recv;
}

// Read `count` consecutive holding registers starting at `addr`.
// Result goes to `out[]`. Caller guarantees out has `count` slots.
static bool modbus_read_registers(uint16_t addr, uint16_t count, uint16_t* out) {
    if (count == 0 || count > 32) return false;
    modbus_drain();

    uint8_t req[8];
    req[0] = ZLAC_SLAVE_ADDR;
    req[1] = FC_READ_HOLDING;
    req[2] = (addr  >> 8) & 0xFF;  req[3] = addr  & 0xFF;
    req[4] = (count >> 8) & 0xFF;  req[5] = count & 0xFF;
    modbus_send(req, 6);

    size_t  expected = 5 + 2 * count;   // slave+fc+bytecount+data+crc(2)
    uint8_t resp[80];
    if (expected > sizeof(resp)) return false;
    if (!modbus_recv(resp, expected, MODBUS_TIMEOUT_MS)) return false;

    if (resp[0] != ZLAC_SLAVE_ADDR)    return false;
    if (resp[1] & 0x80)                return false;  // Modbus exception
    if (resp[1] != FC_READ_HOLDING)    return false;
    if (resp[2] != 2 * count)          return false;

    for (uint16_t i = 0; i < count; i++) {
        out[i] = ((uint16_t)resp[3 + 2*i] << 8) | resp[4 + 2*i];
    }
    return true;
}

static bool modbus_write_single(uint16_t addr, uint16_t value) {
    modbus_drain();

    uint8_t req[8];
    req[0] = ZLAC_SLAVE_ADDR;
    req[1] = FC_WRITE_SINGLE;
    req[2] = (addr  >> 8) & 0xFF;  req[3] = addr  & 0xFF;
    req[4] = (value >> 8) & 0xFF;  req[5] = value & 0xFF;
    modbus_send(req, 6);

    uint8_t resp[8];
    if (!modbus_recv(resp, 8, MODBUS_TIMEOUT_MS)) return false;
    return resp[0] == ZLAC_SLAVE_ADDR &&
           resp[1] == FC_WRITE_SINGLE &&
           resp[2] == req[2] && resp[3] == req[3];
}

// Write `count` consecutive holding registers — used for atomic dual-target RPM.
static bool modbus_write_multiple(uint16_t addr,
                                  const uint16_t* values, uint16_t count) {
    if (count == 0 || count > 16) return false;
    modbus_drain();

    uint8_t req[64];
    req[0] = ZLAC_SLAVE_ADDR;
    req[1] = FC_WRITE_MULTIPLE;
    req[2] = (addr  >> 8) & 0xFF;  req[3] = addr  & 0xFF;
    req[4] = (count >> 8) & 0xFF;  req[5] = count & 0xFF;
    req[6] = 2 * count;
    for (uint16_t i = 0; i < count; i++) {
        req[7 + 2*i] = (values[i] >> 8) & 0xFF;
        req[8 + 2*i] =  values[i]       & 0xFF;
    }
    size_t req_len = 7 + 2 * count;
    modbus_send(req, req_len);

    uint8_t resp[8];
    if (!modbus_recv(resp, 8, MODBUS_TIMEOUT_MS)) return false;
    return resp[0] == ZLAC_SLAVE_ADDR &&
           resp[1] == FC_WRITE_MULTIPLE &&
           resp[2] == req[2] && resp[3] == req[3];
}

// ============================================================================
// ZLAC high-level ops
// ============================================================================

static bool zlac_setup_velocity_mode() {
    if (!modbus_write_single(REG_OPERATION_MODE,   MODE_VELOCITY))    return false;
    if (!modbus_write_single(REG_COMM_WATCHDOG_MS, COMM_WATCHDOG_MS)) return false;
    return true;
}

static bool zlac_enable()      { return modbus_write_single(REG_CONTROL_WORD, CTRL_ENABLE); }
static bool zlac_disable()     { return modbus_write_single(REG_CONTROL_WORD, CTRL_DISABLE); }
static bool zlac_clear_fault() { return modbus_write_single(REG_CONTROL_WORD, CTRL_CLEAR_FAULT); }

static bool zlac_set_target_rpm(int16_t left, int16_t right) {
    uint16_t vals[2] = { (uint16_t)left, (uint16_t)right };
    return modbus_write_multiple(REG_TARGET_RPM_M1, vals, 2);
}

// Read 8 contiguous registers 0x20A5..0x20AC in one transaction:
//   0x20A5 status (unused here)
//   0x20A6 fault
//   0x20A7+0x20A8 position M1 (hi, lo)
//   0x20A9+0x20AA position M2 (hi, lo)
//   0x20AB actual RPM M1
//   0x20AC actual RPM M2
static bool zlac_read_feedback(ZLACFeedback* fb) {
    uint16_t r[8];
    if (!modbus_read_registers(REG_DRIVER_STATUS, 8, r)) return false;
    fb->fault = r[1];
    fb->pos_l = ((int32_t)r[2] << 16) | r[3];
    fb->pos_r = ((int32_t)r[4] << 16) | r[5];
    fb->rpm_l = (int16_t)r[6];
    fb->rpm_r = (int16_t)r[7];
    return true;
}

// ============================================================================
// KINEMATICS — cmd_vel → motor RPM
// ============================================================================

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void cmd_vel_to_target_rpm(float linear, float angular,
                                  int16_t* left_rpm, int16_t* right_rpm) {
    linear  = clampf(linear,  -MAX_LINEAR_VEL,  MAX_LINEAR_VEL);
    angular = clampf(angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

    float v_l = linear - angular * WHEEL_BASE * 0.5f;
    float v_r = linear + angular * WHEEL_BASE * 0.5f;
    v_l = clampf(v_l, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    v_r = clampf(v_r, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);

    float rpm_l = (float)LEFT_SIGN  * v_l * MOTOR_RPM_PER_MPS;
    float rpm_r = (float)RIGHT_SIGN * v_r * MOTOR_RPM_PER_MPS;
    rpm_l = clampf(rpm_l, (float)-MAX_MOTOR_RPM, (float)MAX_MOTOR_RPM);
    rpm_r = clampf(rpm_r, (float)-MAX_MOTOR_RPM, (float)MAX_MOTOR_RPM);

    *left_rpm  = (int16_t)lroundf(rpm_l);
    *right_rpm = (int16_t)lroundf(rpm_r);
}

// ============================================================================
// CONTROL TASK (Core 1, 50 Hz) — deterministic Modbus I/O + state machine
// ============================================================================

static void controlTask(void* pvp) {
    int32_t  prev_pos_l = 0, prev_pos_r = 0;
    int64_t  accum_pos_l = 0, accum_pos_r = 0;
    bool     pos_inited = false;
    uint32_t last_fault_retry_ms = 0;
    uint32_t last_setup_retry_ms = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));

        if (g_state == STATE_INIT || g_state == STATE_WAIT_AGENT) continue;

        uint32_t now = millis();

        CmdShared cmd;
        portENTER_CRITICAL(&g_cmd_mux); cmd = g_cmd; portEXIT_CRITICAL(&g_cmd_mux);

        bool cmd_fresh = (cmd.last_update_ms > 0) &&
                         ((now - cmd.last_update_ms) < CMD_TIMEOUT_MS);

        int16_t tgt_l = 0, tgt_r = 0;

        // --- State machine -------------------------------------------------
        switch (g_state) {
        case STATE_READY:
            if ((now - last_setup_retry_ms) >= SETUP_RETRY_MS) {
                last_setup_retry_ms = now;
                if (zlac_setup_velocity_mode() && zlac_enable()) {
                    g_state = STATE_ACTIVE;
                }
            }
            break;

        case STATE_ACTIVE:
            if (!cmd_fresh) {
                g_state = STATE_SAFE_STOP;
            } else {
                cmd_vel_to_target_rpm(cmd.linear, cmd.angular, &tgt_l, &tgt_r);
            }
            break;

        case STATE_SAFE_STOP:
            // tgt_l, tgt_r already 0 — keep writing zeros to keep ZLAC's
            // own comm-watchdog happy while we wait for fresh commands.
            if (cmd_fresh) {
                g_state = STATE_ACTIVE;
                cmd_vel_to_target_rpm(cmd.linear, cmd.angular, &tgt_l, &tgt_r);
            }
            break;

        case STATE_FAULT:
            if ((now - last_fault_retry_ms) >= FAULT_RETRY_MS) {
                last_fault_retry_ms = now;
                if (zlac_clear_fault()) {
                    g_state = STATE_READY;
                }
            }
            break;

        default: break;
        }

        // --- Modbus I/O ----------------------------------------------------
        bool write_ok = zlac_set_target_rpm(tgt_l, tgt_r);

        ZLACFeedback fb;
        bool read_ok = zlac_read_feedback(&fb);

        uint16_t mb_fails;
        portENTER_CRITICAL(&g_fb_mux); mb_fails = g_fb.modbus_fails; portEXIT_CRITICAL(&g_fb_mux);

        if (write_ok && read_ok) {
            mb_fails = 0;
        } else {
            if (mb_fails < 0xFFFF) mb_fails++;
            if (mb_fails >= MODBUS_MAX_FAILS && g_state == STATE_ACTIVE) {
                g_state = STATE_SAFE_STOP;
            }
        }

        // --- Accumulate positions across int32 wraps -----------------------
        // Apply LEFT_SIGN/RIGHT_SIGN so /wheel_encoders semantics is:
        // "robot moves forward → both values increase".  Without this,
        // the wheel commanded with negative sign (mounted mirror-image)
        // would report decreasing position during forward motion, which
        // makes the downstream odometry compute backward translation
        // plus rotation instead of pure forward motion.
        if (read_ok) {
            if (!pos_inited) {
                pos_inited = true;
            } else {
                accum_pos_l += (int32_t)LEFT_SIGN  * (int32_t)(fb.pos_l - prev_pos_l);
                accum_pos_r += (int32_t)RIGHT_SIGN * (int32_t)(fb.pos_r - prev_pos_r);
            }
            prev_pos_l = fb.pos_l;
            prev_pos_r = fb.pos_r;

            // ZLAC reported a fault — latch FAULT and disable
            if (fb.fault != 0 && g_state != STATE_FAULT) {
                g_state = STATE_FAULT;
                zlac_set_target_rpm(0, 0);
                zlac_disable();
            }
        }

        // --- Publish shared state for microrosTask -------------------------
        // Sign-correct actual RPM (same reason as encoder position above):
        // we want "wheel rotating forward → positive RPM" for both wheels,
        // so it matches the commanded target RPM after sign mapping.
        portENTER_CRITICAL(&g_fb_mux);
        if (read_ok) {
            g_fb.actual_rpm_l = LEFT_SIGN  * fb.rpm_l;
            g_fb.actual_rpm_r = RIGHT_SIGN * fb.rpm_r;
            g_fb.fault_code   = fb.fault;
            g_fb.fresh        = true;
        }
        g_fb.position_l   = accum_pos_l;
        g_fb.position_r   = accum_pos_r;
        g_fb.modbus_fails = mb_fails;
        portEXIT_CRITICAL(&g_fb_mux);

        portENTER_CRITICAL(&g_snap_mux);
        g_snap.target_rpm_l = tgt_l;
        g_snap.target_rpm_r = tgt_r;
        g_snap.state        = (uint8_t)g_state;
        g_snap.cmd_fresh    = cmd_fresh;
        portEXIT_CRITICAL(&g_snap_mux);
    }
}

// ============================================================================
// micro-ROS OBJECTS
// ============================================================================

static rcl_node_t          g_node;
static rclc_support_t      g_support;
static rcl_allocator_t     g_allocator;
static rclc_executor_t     g_executor;

static rcl_subscription_t  g_sub_cmd;
static rcl_publisher_t     g_pub_enc;
static rcl_publisher_t     g_pub_state;
static rcl_publisher_t     g_pub_fault;
static rcl_publisher_t     g_pub_wd;
static rcl_publisher_t     g_pub_rpm;
static rcl_publisher_t     g_pub_mb_fails;

static geometry_msgs__msg__Twist        g_msg_cmd;
static std_msgs__msg__Int64MultiArray   g_msg_enc;
static std_msgs__msg__UInt8             g_msg_state;
static std_msgs__msg__UInt32            g_msg_fault;
static std_msgs__msg__Bool              g_msg_wd;
static std_msgs__msg__Int16MultiArray   g_msg_rpm;
static std_msgs__msg__UInt32            g_msg_mb_fails;

static int64_t g_enc_data[2] = {0, 0};
static int16_t g_rpm_data[2] = {0, 0};

// --------------------------------------------------------- cmd_vel callback
static void cmdVelCb(const void* msgin) {
    const geometry_msgs__msg__Twist* msg =
        (const geometry_msgs__msg__Twist*)msgin;
    uint32_t now = millis();
    portENTER_CRITICAL(&g_cmd_mux);
    g_cmd.linear         = (float)msg->linear.x;
    g_cmd.angular        = (float)msg->angular.z;
    g_cmd.last_update_ms = now;
    portEXIT_CRITICAL(&g_cmd_mux);
}

// --------------------------------------------------- micro-ROS task
static void microrosTask(void* pvp) {
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
    rclc_publisher_init_default(&g_pub_fault, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/chassis/fault");
    rclc_publisher_init_default(&g_pub_wd, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/chassis/cmd_watchdog");
    rclc_publisher_init_best_effort(&g_pub_rpm, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/chassis/wheel_rpm");
    rclc_publisher_init_default(&g_pub_mb_fails, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/chassis/modbus_fails");

    // Static buffers for multi-array payloads
    g_msg_enc.data.data     = g_enc_data;
    g_msg_enc.data.size     = 2;  g_msg_enc.data.capacity = 2;
    g_msg_rpm.data.data     = g_rpm_data;
    g_msg_rpm.data.size     = 2;  g_msg_rpm.data.capacity = 2;

    rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator);
    rclc_executor_add_subscription(&g_executor, &g_sub_cmd, &g_msg_cmd,
                                   cmdVelCb, ON_NEW_DATA);

    g_state = STATE_READY;

    uint32_t last_enc_ms      = 0;
    uint32_t last_state_ms    = 0;
    uint32_t last_fault_ms    = 0;
    uint32_t last_wd_ms       = 0;
    uint32_t last_rpm_ms      = 0;
    uint32_t last_mb_fails_ms = 0;
    uint32_t last_sync_ms     = millis();

    for (;;) {
        rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));
        uint32_t now = millis();

        if ((now - last_sync_ms) > 60000UL) {
            rmw_uros_sync_session(100);
            last_sync_ms = now;
        }

        FeedbackShared fb;
        portENTER_CRITICAL(&g_fb_mux); fb = g_fb; portEXIT_CRITICAL(&g_fb_mux);

        ControlSnapshot snap;
        portENTER_CRITICAL(&g_snap_mux); snap = g_snap; portEXIT_CRITICAL(&g_snap_mux);

        // /wheel_encoders — 50 Hz, only after first successful Modbus read
        if ((now - last_enc_ms) >= 20UL) {
            last_enc_ms = now;
            if (fb.fresh) {
                g_enc_data[0] = fb.position_l;
                g_enc_data[1] = fb.position_r;
                rcl_publish(&g_pub_enc, &g_msg_enc, NULL);
            }
        }

        // /chassis/wheel_rpm — 20 Hz
        if ((now - last_rpm_ms) >= 50UL) {
            last_rpm_ms = now;
            g_rpm_data[0] = fb.actual_rpm_l;
            g_rpm_data[1] = fb.actual_rpm_r;
            rcl_publish(&g_pub_rpm, &g_msg_rpm, NULL);
        }

        // /chassis/state — 2 Hz heartbeat
        if ((now - last_state_ms) >= 500UL) {
            last_state_ms = now;
            g_msg_state.data = snap.state;
            rcl_publish(&g_pub_state, &g_msg_state, NULL);
        }

        // /chassis/fault — 2 Hz
        if ((now - last_fault_ms) >= 500UL) {
            last_fault_ms = now;
            g_msg_fault.data = (uint32_t)fb.fault_code;
            rcl_publish(&g_pub_fault, &g_msg_fault, NULL);
        }

        // /chassis/cmd_watchdog — 2 Hz
        if ((now - last_wd_ms) >= 500UL) {
            last_wd_ms = now;
            g_msg_wd.data = snap.cmd_fresh;
            rcl_publish(&g_pub_wd, &g_msg_wd, NULL);
        }

        // /chassis/modbus_fails — 2 Hz, diagnostic. Consecutive failure count.
        // 0 = comm healthy; growing fast (~50/sec) = no Modbus responses at all.
        if ((now - last_mb_fails_ms) >= 500UL) {
            last_mb_fails_ms = now;
            g_msg_mb_fails.data = (uint32_t)fb.modbus_fails;
            rcl_publish(&g_pub_mb_fails, &g_msg_mb_fails, NULL);
        }

        // LED: ACTIVE solid, SAFE_STOP slow blink, FAULT fast blink, else breathing
        switch ((SystemState)snap.state) {
        case STATE_ACTIVE:    digitalWrite(LED_PIN, HIGH);                  break;
        case STATE_SAFE_STOP: digitalWrite(LED_PIN, (now %  400) <  200);   break;
        case STATE_FAULT:     digitalWrite(LED_PIN, (now %  100) <   50);   break;
        default:              digitalWrite(LED_PIN, (now % 1000) <  500);   break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    UART_MODBUS.begin(UART_MODBUS_BAUD, SERIAL_8N1,
                      UART_MODBUS_RX_PIN, UART_MODBUS_TX_PIN);

    set_microros_transports();
    delay(2000);

    xTaskCreatePinnedToCore(controlTask,  "ctrl", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(microrosTask, "uros", 8192, NULL, 2, NULL, 1);
}

void loop() {
    // All work in FreeRTOS tasks. Keep Arduino loopTask alive but idle.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
