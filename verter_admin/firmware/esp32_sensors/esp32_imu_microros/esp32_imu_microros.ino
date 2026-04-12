/**
 * ESP32 IMU + Ultrasonic Controller with micro-ROS
 *
 * Нативный ROS2 узел на ESP32 для публикации данных IMU и ультразвуковых датчиков.
 *
 * Топики:
 * - Публикация: /imu/data (sensor_msgs/Imu) — 50 Гц
 * - Публикация: /imu/mag_raw (std_msgs/Float32MultiArray) — 10 Гц [x_µT, y_µT, z_µT, heading_deg]
 * - Публикация: /ultrasonic/distances (std_msgs/Float32MultiArray) — ~7 Гц
 *
 * Железо:
 * - Trema IMU 9 DOF V2.0 (Bosch BMX055) через I2C (SDA=21, SCL=22)
 * - 7x HC-SR04 ультразвуковых датчиков
 *
 * Timing:
 * - IMU читается каждый цикл (50 Гц)
 * - Ультразвук: 1 датчик за цикл (round-robin), полный скан 7 шт ≈ 7 Гц
 * - pulseIn таймаут 15000мкс ≈ 2.5м макс дальность
 */

// BMX055_DISABLE_BMM убран — магнитометр включён для логирования.
// Данные публикуются на /imu/mag_raw для оценки уровня помех от моторов.

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <Wire.h>
#include <iarduino_Position_BMX055.h>

// Переопределяем baud rate micro-ROS транспорта (weak-символ из библиотеки)
extern "C" bool arduino_transport_open(struct uxrCustomTransport * transport) {
  Serial.begin(921600);
  return true;
}

// ESP32 LED
#define LED_PIN 2

// I2C пины (IMU)
#define I2C_SDA 21
#define I2C_SCL 22

// Частоты публикации
#define IMU_PUBLISH_MS 20   // 50 Hz
#define MAG_PUBLISH_MS 100  // 10 Hz (для логирования достаточно)

// Калибровка гироскопа (bias estimation при старте)
#define GYRO_CALIBRATION_SAMPLES 500
#define GYRO_CALIBRATION_DELAY_MS 5

// ============================================================================
// HC-SR04 ULTRASONIC SENSORS (7 штук)
// ============================================================================

#define NUM_SENSORS 7
#define US_TIMEOUT_US 15000   // 15мс таймаут ≈ 2.5м макс дальность
#define US_INVALID   -1.0f    // Невалидное значение (датчик не ответил)

// Пины: {trig, echo}
const int sensorPins[NUM_SENSORS][2] = {
  {16, 34},  // Sensor 1: front_center
  {17, 35},  // Sensor 2: front_left_inner
  {18, 32},  // Sensor 3: front_left_outer
  {19, 33},  // Sensor 4: front_right_inner
  {23, 25},  // Sensor 5: front_right_outer
  {26, 27},  // Sensor 6: left
  {14, 12},  // Sensor 7: right
};

// Текущие расстояния (метры)
float distances[NUM_SENSORS];
int currentSensor = 0;  // Индекс текущего датчика (round-robin)

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================================

// Датчики BMX055
iarduino_Position_BMX055 sensorA(BMA);  // Акселерометр
iarduino_Position_BMX055 sensorG(BMG);  // Гироскоп
iarduino_Position_BMX055 sensorM(BMM);  // Магнитометр (для логирования)

// Bias гироскопа
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// micro-ROS
rcl_publisher_t imu_pub;
rcl_publisher_t mag_pub;
rcl_publisher_t us_pub;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray mag_msg;
std_msgs__msg__Float32MultiArray us_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long lastPublish = 0;
unsigned long lastMagPublish = 0;

// ============================================================================
// micro-ROS ERROR HANDLING
// ============================================================================

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) errorLoop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void errorLoop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ============================================================================
// ULTRASONIC SENSOR READING
// ============================================================================

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (duration == 0) {
    return US_INVALID;
  }
  // Скорость звука 343 м/с → 0.000343 м/мкс, туда-обратно /2
  return (duration * 0.000343f) / 2.0f;
}

void setupUltrasonic() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i][0], OUTPUT);   // trig
    pinMode(sensorPins[i][1], INPUT);    // echo
    distances[i] = US_INVALID;
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Инициализация IMU
  sensorA.begin();
  sensorG.begin();
  sensorM.begin();
  sensorA.setScale(BMA_4G);
  sensorG.setScale(BMG_500DPS);
  sensorM.setScale(BMM_REGULAR);      // 0.6 µT noise, default oversampling
  sensorA.setBandwidths(BMA_125Hz);
  sensorG.setBandwidths(BMG_116Hz);
  sensorM.setBandwidths(BMM_10Hz);    // 10 Hz ODR — достаточно для логирования

  // Аппаратная калибровка BMX055
  delay(1000);
  sensorA.setFastOffset();
  sensorG.setFastOffset();
  // Магнитометр: НЕ калибруем автоматически — хотим видеть сырые данные
  // Калибровка делается вручную после оценки помех

  // Программная калибровка гироскопа
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    sensorG.read(BMG_RAD_S);
    sumX += sensorG.axisX;
    sumY += sensorG.axisY;
    sumZ += sensorG.axisZ;
    if (i % 50 == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(GYRO_CALIBRATION_DELAY_MS);
  }
  gyroBiasX = sumX / GYRO_CALIBRATION_SAMPLES;
  gyroBiasY = sumY / GYRO_CALIBRATION_SAMPLES;
  gyroBiasZ = sumZ / GYRO_CALIBRATION_SAMPLES;
  digitalWrite(LED_PIN, LOW);

  // Инициализация ультразвуковых датчиков
  setupUltrasonic();

  // micro-ROS Serial transport
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Ждём агента
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
  digitalWrite(LED_PIN, HIGH);

  // Init support & node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Синхронизация времени с micro_ros_agent (wall-clock).
  // Без этого header.stamp будет в домене millis() (sec≈600),
  // а EKF ожидает wall-clock (sec≈1775000000) и отбрасывает данные.
  rmw_uros_sync_session(1000);

  RCCHECK(rclc_node_init_default(&node, "esp32_imu", "", &support));

  // Publisher: /imu/data
  RCCHECK(rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"
  ));

  // Publisher: /imu/mag_raw (для логирования магнитометра)
  RCCHECK(rclc_publisher_init_default(
    &mag_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/imu/mag_raw"
  ));

  // Publisher: /ultrasonic/distances
  RCCHECK(rclc_publisher_init_default(
    &us_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/ultrasonic/distances"
  ));

  // Init messages
  initImuMessage();
  initMagMessage();
  initUltrasonicMessage();

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

// ============================================================================
// MESSAGE INITIALIZATION
// ============================================================================

void initImuMessage() {
  imu_msg.header.frame_id.data = (char*)malloc(20);
  imu_msg.header.frame_id.capacity = 20;
  snprintf(imu_msg.header.frame_id.data, 20, "imu_link");
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
  imu_msg.orientation_covariance[0] = -1.0;

  memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;

  memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));
  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;
}

void initMagMessage() {
  // [0]=mag_x (µT), [1]=mag_y (µT), [2]=mag_z (µT), [3]=heading (deg)
  mag_msg.data.capacity = 4;
  mag_msg.data.size = 4;
  mag_msg.data.data = (float*)malloc(4 * sizeof(float));
  for (int i = 0; i < 4; i++) {
    mag_msg.data.data[i] = 0.0f;
  }
}

void initUltrasonicMessage() {
  us_msg.data.capacity = NUM_SENSORS;
  us_msg.data.size = NUM_SENSORS;
  us_msg.data.data = (float*)malloc(NUM_SENSORS * sizeof(float));
  for (int i = 0; i < NUM_SENSORS; i++) {
    us_msg.data.data[i] = US_INVALID;
  }
}

// ============================================================================
// LOOP
// ============================================================================

static unsigned long lastTimeSync = 0;

void loop() {
  unsigned long now = millis();

  // Ре-синхронизация wall-clock каждые 60с (компенсация drift кварца ESP32)
  if (now - lastTimeSync > 60000) {
    rmw_uros_sync_session(100);
    lastTimeSync = now;
  }

  // Читаем IMU + магнитометр (I2C)
  sensorA.read(BMA_M_S);
  sensorG.read(BMG_RAD_S);
  sensorM.read(BMM_MCT);  // Микротесла

  // Читаем ОДИН ультразвуковой датчик (round-robin)
  distances[currentSensor] = readUltrasonic(
    sensorPins[currentSensor][0],
    sensorPins[currentSensor][1]
  );
  currentSensor++;

  // Когда все 7 прочитаны — публикуем и начинаем заново
  if (currentSensor >= NUM_SENSORS) {
    currentSensor = 0;

    // Заполняем сообщение
    for (int i = 0; i < NUM_SENSORS; i++) {
      us_msg.data.data[i] = distances[i];
    }
    RCSOFTCHECK(rcl_publish(&us_pub, &us_msg, NULL));
  }

  // Process micro-ROS callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Публикация IMU с интервалом
  if (now - lastPublish >= IMU_PUBLISH_MS) {
    lastPublish = now;

    // Wall-clock время, синхронизированное с agent через rmw_uros_sync_session().
    int64_t time_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

    imu_msg.angular_velocity.x = sensorG.axisX - gyroBiasX;
    imu_msg.angular_velocity.y = sensorG.axisY - gyroBiasY;
    imu_msg.angular_velocity.z = sensorG.axisZ - gyroBiasZ;

    imu_msg.linear_acceleration.x = sensorA.axisX;
    imu_msg.linear_acceleration.y = sensorA.axisY;
    imu_msg.linear_acceleration.z = sensorA.axisZ;

    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  }

  // Публикация магнитометра (10 Hz)
  if (now - lastMagPublish >= MAG_PUBLISH_MS) {
    lastMagPublish = now;

    mag_msg.data.data[0] = sensorM.axisX;  // µT
    mag_msg.data.data[1] = sensorM.axisY;  // µT
    mag_msg.data.data[2] = sensorM.axisZ;  // µT
    mag_msg.data.data[3] = atan2(sensorM.axisY, sensorM.axisX) * 180.0f / M_PI;  // heading deg

    RCSOFTCHECK(rcl_publish(&mag_pub, &mag_msg, NULL));
  }

  delay(1);
}
