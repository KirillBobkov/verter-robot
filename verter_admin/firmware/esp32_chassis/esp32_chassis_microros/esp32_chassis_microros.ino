/**
 * ESP32 Chassis Controller with micro-ROS
 *
 * Нативный ROS2 узел на ESP32.
 *
 * Топики:
 * - Подписка: /cmd_vel (geometry_msgs/Twist)
 * - Публикация: /wheel_encoders (std_msgs/Int64MultiArray) [left, right, timestamp_ms]
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
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <Wire.h>

// ESP32 не имеет LED_PIN по умолчанию
#define LED_PIN 2

// ============================================================================
// PIN CONFIGURATION (согласно реальной пайке)
// ============================================================================

// Левый мотор (Cytron PWM+DIR)
#define MOTOR_L_PWM   18
#define MOTOR_L_DIR   25

// Правый мотор (Cytron PWM+DIR)
#define MOTOR_R_PWM   19
#define MOTOR_R_DIR   27

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

const float WHEEL_CIRCUMFERENCE = 0.59;
const float GEAR_RATIO = 4.0007;
const float WHEEL_BASE = 0.372;

const float MAX_VELOCITY = 0.5;
const int MAX_PWM = 200;
const int MIN_PWM = 25;
const unsigned long CMD_TIMEOUT = 500;

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

// I2C шины перепутаны физически
WheelControl leftWheel = {0, 0, true};    // useWire1 = true
WheelControl rightWheel = {0, 0, false};  // useWire1 = false

// Velocity mode
bool velocityMode = false;
unsigned long lastCmdTime = 0;
int velocityLeftPWM = 0;
int velocityRightPWM = 0;
unsigned long lastEncoderPublish = 0;

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setMotorLeft(int pwm) {
  if (pwm > 0) {
    digitalWrite(MOTOR_L_DIR, HIGH);
    ledcWrite(PWM_CHANNEL_L, min(pwm, MAX_PWM));
  } else if (pwm < 0) {
    digitalWrite(MOTOR_L_DIR, LOW);
    ledcWrite(PWM_CHANNEL_L, min(-pwm, MAX_PWM));
  } else {
    ledcWrite(PWM_CHANNEL_L, 0);
  }
}

void setMotorRight(int pwm) {
  // Правый мотор инвертирован
  if (pwm > 0) {
    digitalWrite(MOTOR_R_DIR, LOW);
    ledcWrite(PWM_CHANNEL_R, min(pwm, MAX_PWM));
  } else if (pwm < 0) {
    digitalWrite(MOTOR_R_DIR, HIGH);
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

int16_t readAngle(WheelControl* wheel) {
  TwoWire* wire = wheel->useWire1 ? &Wire1_custom : &Wire;

  wire->beginTransmission(AS5600_ADDRESS);
  wire->write(RAW_ANGLE_REG);
  if (wire->endTransmission(false) != 0) return -1;

  if (wire->requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2) == 2) {
    int16_t angle = wire->read() << 8;
    angle |= wire->read();
    return angle & 0x0FFF;
  }
  return -1;
}

void updateEncoder(WheelControl* wheel) {
  int16_t angle = readAngle(wheel);
  if (angle == -1) return;

  int16_t diff = angle - wheel->prevAngle;
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;

  if (wheel->useWire1) {
    wheel->totalSteps += diff;
  } else {
    wheel->totalSteps -= diff;
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
  Wire1_custom.begin(I2C1_SDA, I2C1_SCL);
  Wire1_custom.setClock(400000);

  delay(100);
  leftWheel.prevAngle = readAngle(&leftWheel);
  rightWheel.prevAngle = readAngle(&rightWheel);

  // micro-ROS Serial transport (использует default_transport.cpp)
  // Serial.begin() вызывается внутри set_microros_transports()
  set_microros_transports();

  delay(2000);

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
  RCCHECK(rclc_publisher_init_default(
    &encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "/wheel_encoders"
  ));

  // Init encoder message (3 base + 4 debug)
  encoder_msg.data.capacity = 7;
  encoder_msg.data.size = 7;
  encoder_msg.data.data = (int64_t*)malloc(7 * sizeof(int64_t));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // Read encoders
  updateEncoder(&leftWheel);
  updateEncoder(&rightWheel);

  // Process micro-ROS callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

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
  }

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
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));
  }

  delay(1);
}
