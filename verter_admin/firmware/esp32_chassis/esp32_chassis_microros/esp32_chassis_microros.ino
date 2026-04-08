/**
 * ESP32 Chassis Controller with micro-ROS
 *
 * Нативный ROS2 узел на ESP32.
 *
 * Топики:
 * - Подписка: /cmd_vel (geometry_msgs/Twist)
 * - Публикация: /wheel_encoders (std_msgs/Int64MultiArray)
 *   [left, right, timestamp_ms, pwm_left, pwm_right, vel_left_x10000,
 *    vel_right_x10000, prev_loop_dt_ms, prev_read_left_ms,
 *    prev_read_right_ms, prev_spin_ms, prev_publish_ms, seq, boot_id,
 *    reset_reason, prev_spin_rc, prev_publish_rc,
 *    transport_write_drop_count, transport_short_write_count,
 *    transport_last_available, transport_last_required, spin_error_count,
 *    publish_error_count, left_i2c_tx_fail_count,
 *    left_i2c_short_read_count, right_i2c_tx_fail_count,
 *    right_i2c_short_read_count, encoder_jump_reject_count,
 *    long_loop_count, max_loop_dt_ms, cmd_timeout_stop_count,
 *    prev_cmd_age_ms, free_heap_bytes, min_free_heap_bytes,
 *    velocity_mode]
 *
 * Железо:
 * - Cytron MD10C драйвер (PWM + DIR)
 * - AS5600 магнитные энкодеры через I2C (2 шины)
 *
 * Установка micro_ros_arduino:
 * 1. Скачай https://github.com/micro-ROS/micro_ros_arduino/releases
 * 2. Arduino IDE -> Sketch -> Include Library -> Add .ZIP Library
 */

#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_system.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <Wire.h>

static volatile uint32_t g_transport_write_drop_count = 0;
static volatile uint32_t g_transport_short_write_count = 0;
static volatile uint32_t g_transport_last_available = 0;
static volatile uint32_t g_transport_last_required = 0;

// Forward declarations — Arduino IDE генерирует прототипы функций
// ДО определения структур, поэтому без этих строк не компилируется
struct WheelControl;
struct PIDState;

// Переопределяем micro-ROS транспорт (weak-символы из библиотеки)
extern "C" bool arduino_transport_open(struct uxrCustomTransport * transport) {
  Serial.setRxBufferSize(4096);  // увеличиваем RX буфер (дефолт 256)
  Serial.begin(921600);
  return true;
}

// Ограничиваем таймаут чтения — DDS иногда ставит 1000мс
extern "C" size_t arduino_transport_read(struct uxrCustomTransport * transport,
    uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
  (void)errcode;
  Serial.setTimeout(timeout > 10 ? 10 : timeout);
  return Serial.readBytes((char *)buf, len);
}

// Неблокирующая запись — если TX буфер полный, отбрасываем вместо блокировки
extern "C" size_t arduino_transport_write(struct uxrCustomTransport * transport,
    const uint8_t *buf, size_t len, uint8_t *errcode) {
  (void)errcode;
  size_t available = Serial.availableForWrite();
  if (available < len) {
    // Буфер полный — ждём не более 5мс
    unsigned long start = millis();
    while (Serial.availableForWrite() < len && (millis() - start) < 5) {
      delayMicroseconds(100);
    }
    available = Serial.availableForWrite();
    if (available < len) {
      g_transport_write_drop_count++;
      g_transport_last_available = available;
      g_transport_last_required = len;
      return 0;  // отбросить пакет вместо блокировки на 1с
    }
  }
  size_t written = Serial.write(buf, len);
  if (written < len) {
    g_transport_short_write_count++;
    g_transport_last_available = available;
    g_transport_last_required = len;
  }
  return written;
}

// ESP32 не имеет LED_PIN по умолчанию
#define LED_PIN 2

// ============================================================================
// PIN CONFIGURATION (согласно реальной пайке)
// ============================================================================

// Левый мотор (Cytron PWM+DIR) — проверка: моторы могут быть перекрёстно подключены
#define MOTOR_L_PWM   19
#define MOTOR_L_DIR   27

// Правый мотор (Cytron PWM+DIR)
#define MOTOR_R_PWM   18
#define MOTOR_R_DIR   25

// Левый энкодер AS5600 (I2C1 - Wire1) - перепутаны физически
#define I2C0_SDA      21
#define I2C0_SCL      22

// Правый энкодер AS5600 (I2C0 - Wire)
#define I2C1_SDA      32
#define I2C1_SCL      4

// PWM конфигурация ESP32
#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_CHANNEL_L   0
#define PWM_CHANNEL_R   1

// AS5600
#define AS5600_ADDRESS  0x36
#define RAW_ANGLE_REG   0x0C

// ============================================================================
// ПАРАМЕТРЫ РОБОТА
// ============================================================================

// Калибровка направления.
// Положительный /cmd_vel.x должен означать:
// - оба мотора крутят колёса вперёд;
// - оба totalSteps растут.
const uint8_t LEFT_MOTOR_FORWARD_DIR_LEVEL = LOW;
const uint8_t RIGHT_MOTOR_FORWARD_DIR_LEVEL = HIGH;
const int LEFT_ENCODER_FORWARD_SIGN = -1;
const int RIGHT_ENCODER_FORWARD_SIGN = 1;

const float WHEEL_CIRCUMFERENCE = 0.576;
const float GEAR_RATIO = 4.0007;
const float WHEEL_BASE = 0.378;

const float MAX_VELOCITY = 0.5;
const int MAX_PWM = 200;
const int MIN_PWM = 25;
const unsigned long CMD_TIMEOUT = 1500;  // 1.5с — увеличено чтобы моторы не стопились при 1с keepalive XRCE-DDS

const float ENCODER_RESOLUTION = 4096.0;
const float METERS_PER_STEP = WHEEL_CIRCUMFERENCE / (ENCODER_RESOLUTION * GEAR_RATIO);

// PID
const float PID_KP = 80.0;
const float PID_KI = 50.0;
const float PID_KD = 5.0;
const float PID_MAX_INTEGRAL = 100.0;
const unsigned long PID_INTERVAL = 50;
const int MAX_PWM_CHANGE = 15;

#define ENCODER_PUBLISH_MS 50
#define LONG_LOOP_THRESHOLD_MS 200
#define ENCODER_MSG_SIZE 35

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================================

TwoWire Wire1_custom = TwoWire(1);

// micro-ROS
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t encoder_pub;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int64MultiArray encoder_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// PID
struct PIDState {
  float desiredVelocity;
  float actualVelocity;
  float error;
  float lastError;
  float integral;
  int outputPWM;
  long lastSteps;
  unsigned long lastTime;
};

PIDState leftPID = {0, 0, 0, 0, 0, 0, 0, 0};
PIDState rightPID = {0, 0, 0, 0, 0, 0, 0, 0};

// Колёса
struct WheelControl {
  int16_t prevAngle;
  long totalSteps;
  bool useWire1;
};

struct LoopStats {
  unsigned long loopDtMs;
  unsigned long readLeftMs;
  unsigned long readRightMs;
  unsigned long spinMs;
  unsigned long publishMs;
  unsigned long cmdAgeMs;
  int32_t spinRc;
  int32_t publishRc;
};

// Левый энкодер на Wire (I2C0, SDA=21, SCL=22)
// Правый энкодер на Wire1_custom (I2C1, SDA=32, SCL=4)
WheelControl leftWheel = {0, 0, false};   // useWire1 = false → Wire (21,22)
WheelControl rightWheel = {0, 0, true};   // useWire1 = true  → Wire1_custom (32,4)

// Velocity mode
bool velocityMode = false;
unsigned long lastCmdTime = 0;
int velocityLeftPWM = 0;
int velocityRightPWM = 0;
unsigned long lastEncoderPublish = 0;
unsigned long lastLoopTime = 0;
unsigned long lastLoopDtMs = 0;
unsigned long lastReadLeftDtMs = 0;
unsigned long lastReadRightDtMs = 0;
unsigned long lastSpinDtMs = 0;
unsigned long lastPublishDtMs = 0;
LoopStats currentLoopStats = {0, 0, 0, 0, 0, 0, 0, 0};
LoopStats previousLoopStats = {0, 0, 0, 0, 0, 0, 0, 0};
RTC_DATA_ATTR uint32_t rtcBootCounter = 0;
uint32_t bootId = 0;
int32_t bootResetReason = 0;
uint32_t packetSequence = 0;
uint32_t spinErrorCount = 0;
uint32_t publishErrorCount = 0;
uint32_t leftI2cTxFailCount = 0;
uint32_t leftI2cShortReadCount = 0;
uint32_t rightI2cTxFailCount = 0;
uint32_t rightI2cShortReadCount = 0;
uint32_t encoderJumpRejectCount = 0;
uint32_t longLoopCount = 0;
uint32_t maxLoopDtMs = 0;
uint32_t cmdTimeoutStopCount = 0;

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setMotorLeft(int pwm) {
  if (pwm > 0) {
    digitalWrite(MOTOR_L_DIR, LEFT_MOTOR_FORWARD_DIR_LEVEL);
    ledcWrite(PWM_CHANNEL_L, min(pwm, MAX_PWM));
  } else if (pwm < 0) {
    digitalWrite(MOTOR_L_DIR, !LEFT_MOTOR_FORWARD_DIR_LEVEL);
    ledcWrite(PWM_CHANNEL_L, min(-pwm, MAX_PWM));
  } else {
    ledcWrite(PWM_CHANNEL_L, 0);
  }
}

void setMotorRight(int pwm) {
  if (pwm > 0) {
    digitalWrite(MOTOR_R_DIR, RIGHT_MOTOR_FORWARD_DIR_LEVEL);
    ledcWrite(PWM_CHANNEL_R, min(pwm, MAX_PWM));
  } else if (pwm < 0) {
    digitalWrite(MOTOR_R_DIR, !RIGHT_MOTOR_FORWARD_DIR_LEVEL);
    ledcWrite(PWM_CHANNEL_R, min(-pwm, MAX_PWM));
  } else {
    ledcWrite(PWM_CHANNEL_R, 0);
  }
}

void stopMotors() {
  ledcWrite(PWM_CHANNEL_L, 0);
  ledcWrite(PWM_CHANNEL_R, 0);
}

// ============================================================================
// ENCODERS
// ============================================================================

int16_t readAngle(
  WheelControl* wheel,
  uint32_t* txFailCount,
  uint32_t* shortReadCount
) {
  TwoWire* wire = wheel->useWire1 ? &Wire1_custom : &Wire;

  wire->beginTransmission(AS5600_ADDRESS);
  wire->write(RAW_ANGLE_REG);
  // true = STOP вместо REPEATED START.
  // ESP32 I2C_NUM_0 зависает на repeated start (известный баг кремния).
  // AS5600 нормально работает через STOP + новый START.
  if (wire->endTransmission(true) != 0) {
    (*txFailCount)++;
    return -1;
  }

  if (wire->requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2) == 2) {
    int16_t angle = wire->read() << 8;
    angle |= wire->read();
    return angle & 0x0FFF;
  }
  (*shortReadCount)++;
  return -1;
}

void updateEncoder(
  WheelControl* wheel,
  uint32_t* txFailCount,
  uint32_t* shortReadCount
) {
  int16_t angle = readAngle(wheel, txFailCount, shortReadCount);
  if (angle == -1) return;

  int16_t diff = angle - wheel->prevAngle;
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;

  // Reject I2C corrupted reads: at max motor speed (~3.3 rev/s)
  // and typical loop ~20ms, max plausible diff ≈ 270 steps.
  // 350 covers up to ~30ms loop at max speed.
  if (diff > 350 || diff < -350) {
    encoderJumpRejectCount++;
    return;
  }

  // Нормализуем знак так, чтобы движение вперёд всегда увеличивало totalSteps.
  if (wheel->useWire1) {
    wheel->totalSteps += RIGHT_ENCODER_FORWARD_SIGN * diff;  // правый энкодер
  } else {
    wheel->totalSteps += LEFT_ENCODER_FORWARD_SIGN * diff;   // левый энкодер
  }
  wheel->prevAngle = angle;
}

// ============================================================================
// PID
// ============================================================================

void calculateWheelVelocity(PIDState* pid, long currentSteps) {
  unsigned long now = millis();
  unsigned long dt = now - pid->lastTime;
  if (dt == 0) return;

  long deltaSteps = currentSteps - pid->lastSteps;
  float deltaMeters = deltaSteps * METERS_PER_STEP;
  pid->actualVelocity = deltaMeters / (dt / 1000.0);
  pid->lastSteps = currentSteps;
  pid->lastTime = now;
}

int computePID(PIDState* pid) {
  pid->error = pid->desiredVelocity - pid->actualVelocity;
  pid->integral = constrain(pid->integral + pid->error, -PID_MAX_INTEGRAL, PID_MAX_INTEGRAL);
  float derivative = pid->error - pid->lastError;
  pid->lastError = pid->error;

  float output = PID_KP * pid->error + PID_KI * pid->integral + PID_KD * derivative;
  pid->outputPWM = (int)output;

  if (abs(pid->desiredVelocity) > 0.01 && pid->outputPWM != 0 && abs(pid->outputPWM) < MIN_PWM) {
    pid->outputPWM = (pid->outputPWM > 0) ? MIN_PWM : -MIN_PWM;
  }
  return constrain(pid->outputPWM, -MAX_PWM, MAX_PWM);
}

void resetPID(PIDState* pid) {
  pid->error = 0;
  pid->lastError = 0;
  pid->integral = 0;
  pid->outputPWM = 0;
  pid->actualVelocity = 0;
}

void updatePID() {
  unsigned long now = millis();
  if (now - leftPID.lastTime < PID_INTERVAL) return;

  calculateWheelVelocity(&leftPID, leftWheel.totalSteps);
  calculateWheelVelocity(&rightPID, rightWheel.totalSteps);

  int targetLeftPWM = computePID(&leftPID);
  int targetRightPWM = computePID(&rightPID);

  int leftChange = constrain(targetLeftPWM - velocityLeftPWM, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
  int rightChange = constrain(targetRightPWM - velocityRightPWM, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);

  velocityLeftPWM += leftChange;
  velocityRightPWM += rightChange;

  // Стартовый порог: если PID хочет ехать а PWM в мёртвой зоне — сразу прыгаем на MIN_PWM
  if (abs(leftPID.desiredVelocity) > 0.01 && velocityLeftPWM != 0 && abs(velocityLeftPWM) < MIN_PWM) {
    velocityLeftPWM = (velocityLeftPWM > 0) ? MIN_PWM : -MIN_PWM;
  }
  if (abs(rightPID.desiredVelocity) > 0.01 && velocityRightPWM != 0 && abs(velocityRightPWM) < MIN_PWM) {
    velocityRightPWM = (velocityRightPWM > 0) ? MIN_PWM : -MIN_PWM;
  }

  setMotorLeft(velocityLeftPWM);
  setMotorRight(velocityRightPWM);
}

// ============================================================================
// VELOCITY CONTROL
// ============================================================================

void processVelocity(float linear, float angular) {
  lastCmdTime = millis();

  if (abs(linear) < 0.01 && abs(angular) < 0.01) {
    velocityMode = false;
    velocityLeftPWM = 0;
    velocityRightPWM = 0;
    resetPID(&leftPID);
    resetPID(&rightPID);
    stopMotors();
    return;
  }

  float leftVel = linear - (angular * WHEEL_BASE / 2.0);
  float rightVel = linear + (angular * WHEEL_BASE / 2.0);

  float maxWheelVel = max(abs(leftVel), abs(rightVel));
  if (maxWheelVel > MAX_VELOCITY) {
    leftVel = leftVel * MAX_VELOCITY / maxWheelVel;
    rightVel = rightVel * MAX_VELOCITY / maxWheelVel;
  }

  if (!velocityMode) {
    resetPID(&leftPID);
    resetPID(&rightPID);
    leftPID.lastSteps = leftWheel.totalSteps;
    rightPID.lastSteps = rightWheel.totalSteps;
    leftPID.lastTime = millis();
    rightPID.lastTime = millis();
  }

  leftPID.desiredVelocity = leftVel;
  rightPID.desiredVelocity = rightVel;
  velocityMode = true;
}

// ============================================================================
// micro-ROS CALLBACK
// ============================================================================

void cmdVelCallback(const void* msg_in) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
  processVelocity(msg->linear.x, msg->angular.z);
}

// ============================================================================
// micro-ROS ERROR HANDLING
// ============================================================================

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) errorLoop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void errorLoop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  bootId = ++rtcBootCounter;
  bootResetReason = static_cast<int32_t>(esp_reset_reason());

  // Отключаем WiFi и Bluetooth — их FreeRTOS задачи периодически блокируют CPU на ~1с
  WiFi.mode(WIFI_OFF);
  esp_bt_controller_disable();

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Motors
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_PWM, PWM_CHANNEL_L);
  ledcAttachPin(MOTOR_R_PWM, PWM_CHANNEL_R);
  stopMotors();

  // I2C
  Wire.begin(I2C0_SDA, I2C0_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  Wire1_custom.begin(I2C1_SDA, I2C1_SCL);
  Wire1_custom.setClock(400000);
  Wire1_custom.setTimeOut(20);

  delay(100);
  leftWheel.prevAngle = readAngle(&leftWheel, &leftI2cTxFailCount, &leftI2cShortReadCount);
  rightWheel.prevAngle = readAngle(&rightWheel, &rightI2cTxFailCount, &rightI2cShortReadCount);

  // micro-ROS Serial transport
  set_microros_transports();

  delay(2000);
  lastLoopTime = millis();

  allocator = rcl_get_default_allocator();

  // Wait for agent (LED blinks)
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
  digitalWrite(LED_PIN, HIGH);  // Connected

  // Init support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_chassis", "", &support));

  // Subscriber: /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  ));

  // Publisher: /wheel_encoders
  // Sensor telemetry should prefer freshness over reliable delivery retries
  // that can stall the main loop on a noisy serial micro-ROS transport.
  rmw_qos_profile_t wheel_encoders_qos = rmw_qos_profile_sensor_data;
  wheel_encoders_qos.depth = 10;
  RCCHECK(rclc_publisher_init(
    &encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "/wheel_encoders",
    &wheel_encoders_qos
  ));

  // Init encoder message (base fields + extended diagnostics)
  encoder_msg.data.capacity = ENCODER_MSG_SIZE;
  encoder_msg.data.size = ENCODER_MSG_SIZE;
  encoder_msg.data.data = (int64_t*)malloc(ENCODER_MSG_SIZE * sizeof(int64_t));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();
  unsigned long stageStart = 0;
  lastLoopDtMs = now - lastLoopTime;
  lastLoopTime = now;
  currentLoopStats.loopDtMs = lastLoopDtMs;
  currentLoopStats.readLeftMs = 0;
  currentLoopStats.readRightMs = 0;
  currentLoopStats.spinMs = 0;
  currentLoopStats.publishMs = 0;
  currentLoopStats.cmdAgeMs = 0;
  currentLoopStats.spinRc = 0;
  currentLoopStats.publishRc = 0;

  // Read encoders
  stageStart = millis();
  updateEncoder(&leftWheel, &leftI2cTxFailCount, &leftI2cShortReadCount);
  lastReadLeftDtMs = millis() - stageStart;
  currentLoopStats.readLeftMs = lastReadLeftDtMs;
  stageStart = millis();
  updateEncoder(&rightWheel, &rightI2cTxFailCount, &rightI2cShortReadCount);
  lastReadRightDtMs = millis() - stageStart;
  currentLoopStats.readRightMs = lastReadRightDtMs;

  // Process micro-ROS callbacks
  stageStart = millis();
  rcl_ret_t spinRc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  lastSpinDtMs = millis() - stageStart;
  currentLoopStats.spinMs = lastSpinDtMs;
  currentLoopStats.spinRc = static_cast<int32_t>(spinRc);
  if (spinRc != RCL_RET_OK) {
    spinErrorCount++;
  }

  // PID control
  if (velocityMode) {
    updatePID();
  }

  // Safety timeout (используем millis() а не now, т.к. lastCmdTime обновляется в callback)
  if (velocityMode && (millis() - lastCmdTime > CMD_TIMEOUT)) {
    velocityMode = false;
    velocityLeftPWM = 0;
    velocityRightPWM = 0;
    resetPID(&leftPID);
    resetPID(&rightPID);
    stopMotors();
    cmdTimeoutStopCount++;
  }
  currentLoopStats.cmdAgeMs = velocityMode ? (millis() - lastCmdTime) : 0;

  // Publish encoders
  if (now - lastEncoderPublish >= ENCODER_PUBLISH_MS) {
    lastEncoderPublish = now;
    encoder_msg.data.data[0] = leftWheel.totalSteps;
    encoder_msg.data.data[1] = rightWheel.totalSteps;
    encoder_msg.data.data[2] = now;
    encoder_msg.data.data[3] = velocityLeftPWM;                          // текущий PWM левый
    encoder_msg.data.data[4] = velocityRightPWM;                         // текущий PWM правый
    encoder_msg.data.data[5] = (int64_t)(leftPID.actualVelocity * 10000);  // actual vel * 10000
    encoder_msg.data.data[6] = (int64_t)(rightPID.actualVelocity * 10000); // actual vel * 10000
    encoder_msg.data.data[7] = previousLoopStats.loopDtMs;              // dt завершённого предыдущего loop()
    encoder_msg.data.data[8] = previousLoopStats.readLeftMs;            // чтение левого AS5600
    encoder_msg.data.data[9] = previousLoopStats.readRightMs;           // чтение правого AS5600
    encoder_msg.data.data[10] = previousLoopStats.spinMs;               // spin_some executor
    encoder_msg.data.data[11] = previousLoopStats.publishMs;            // publish предыдущего пакета
    encoder_msg.data.data[12] = ++packetSequence;                       // последовательность пакетов
    encoder_msg.data.data[13] = bootId;                                 // boot id в RTC памяти
    encoder_msg.data.data[14] = bootResetReason;                        // esp_reset_reason()
    encoder_msg.data.data[15] = previousLoopStats.spinRc;               // rc spin_some
    encoder_msg.data.data[16] = previousLoopStats.publishRc;            // rc publish
    encoder_msg.data.data[17] = g_transport_write_drop_count;           // transport write dropped
    encoder_msg.data.data[18] = g_transport_short_write_count;          // partial transport writes
    encoder_msg.data.data[19] = g_transport_last_available;             // availableForWrite() at anomaly
    encoder_msg.data.data[20] = g_transport_last_required;              // bytes required at anomaly
    encoder_msg.data.data[21] = spinErrorCount;                         // non-OK spin rc count
    encoder_msg.data.data[22] = publishErrorCount;                      // non-OK publish rc count
    encoder_msg.data.data[23] = leftI2cTxFailCount;                     // left endTransmission failures
    encoder_msg.data.data[24] = leftI2cShortReadCount;                  // left short reads
    encoder_msg.data.data[25] = rightI2cTxFailCount;                    // right endTransmission failures
    encoder_msg.data.data[26] = rightI2cShortReadCount;                 // right short reads
    encoder_msg.data.data[27] = encoderJumpRejectCount;                 // implausible encoder diffs
    encoder_msg.data.data[28] = longLoopCount;                          // loop_dt > threshold count
    encoder_msg.data.data[29] = maxLoopDtMs;                            // maximum observed loop dt
    encoder_msg.data.data[30] = cmdTimeoutStopCount;                    // motor safety timeout count
    encoder_msg.data.data[31] = previousLoopStats.cmdAgeMs;             // age of last /cmd_vel in previous loop
    encoder_msg.data.data[32] = ESP.getFreeHeap();                      // current free heap
    encoder_msg.data.data[33] = ESP.getMinFreeHeap();                   // min free heap watermark
    encoder_msg.data.data[34] = velocityMode ? 1 : 0;                   // current velocity mode flag
    stageStart = millis();
    rcl_ret_t publishRc = rcl_publish(&encoder_pub, &encoder_msg, NULL);
    lastPublishDtMs = millis() - stageStart;
    currentLoopStats.publishMs = lastPublishDtMs;
    currentLoopStats.publishRc = static_cast<int32_t>(publishRc);
    if (publishRc != RCL_RET_OK) {
      publishErrorCount++;
    }
  }

  if (currentLoopStats.loopDtMs > LONG_LOOP_THRESHOLD_MS) {
    longLoopCount++;
  }
  if (currentLoopStats.loopDtMs > maxLoopDtMs) {
    maxLoopDtMs = currentLoopStats.loopDtMs;
  }
  previousLoopStats = currentLoopStats;

  delay(1);
}
