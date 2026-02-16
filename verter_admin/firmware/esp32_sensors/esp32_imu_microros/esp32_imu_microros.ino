/**
 * ESP32 IMU Controller with micro-ROS
 *
 * Нативный ROS2 узел на ESP32 для публикации данных IMU.
 *
 * Топики:
 * - Публикация: /imu/data (sensor_msgs/Imu)
 *
 * Железо:
 * - Trema IMU 9 DOF V2.0 (Bosch BMX055) через I2C
 * - SDA = GPIO 21, SCL = GPIO 22
 *
 * Данные:
 * - linear_acceleration (m/s², включая гравитацию)
 * - angular_velocity (rad/s)
 * - orientation: не публикуется (covariance[0] = -1)
 *
 * Установка:
 * 1. micro_ros_arduino v2.0.8-humble (та же что и для chassis)
 * 2. iarduino_Position_BMX055 библиотека
 */

#define BMX055_DISABLE_BMM  // Магнитометр отключён (помехи от моторов)

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <Wire.h>
#include <iarduino_Position_BMX055.h>

// ESP32 LED
#define LED_PIN 2

// I2C пины
#define I2C_SDA 21
#define I2C_SCL 22

// Частота публикации
#define IMU_PUBLISH_MS 20  // 50 Hz

// Калибровка гироскопа (bias estimation при старте)
#define GYRO_CALIBRATION_SAMPLES 500  // ~2.5 сек при 5ms задержке
#define GYRO_CALIBRATION_DELAY_MS 5

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================================

// Датчики BMX055 (раздельные объекты для raw данных)
iarduino_Position_BMX055 sensorA(BMA);  // Акселерометр
iarduino_Position_BMX055 sensorG(BMG);  // Гироскоп

// Bias гироскопа (вычисляется при старте)
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// micro-ROS
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long lastPublish = 0;

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
// SETUP
// ============================================================================

void setup() {
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Инициализация датчиков
  sensorA.begin();
  sensorG.begin();

  // Настройка диапазонов
  sensorA.setScale(BMA_4G);       // +-4g (достаточно для мобильного робота)
  sensorG.setScale(BMG_500DPS);   // +-500 deg/s

  // Частота обновления данных
  sensorA.setBandwidths(BMA_125Hz);
  sensorG.setBandwidths(BMG_116Hz);

  // Аппаратная быстрая калибровка BMX055
  delay(1000);
  sensorA.setFastOffset();
  sensorG.setFastOffset();

  // Программная калибровка гироскопа — робот должен быть неподвижен!
  // Собираем GYRO_CALIBRATION_SAMPLES сэмплов и вычисляем средний bias
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    sensorG.read(BMG_RAD_S);
    sumX += sensorG.axisX;
    sumY += sensorG.axisY;
    sumZ += sensorG.axisZ;
    // LED мигает во время калибровки
    if (i % 50 == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(GYRO_CALIBRATION_DELAY_MS);
  }
  gyroBiasX = sumX / GYRO_CALIBRATION_SAMPLES;
  gyroBiasY = sumY / GYRO_CALIBRATION_SAMPLES;
  gyroBiasZ = sumZ / GYRO_CALIBRATION_SAMPLES;
  digitalWrite(LED_PIN, LOW);  // Калибровка завершена

  // micro-ROS Serial transport
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Ждём агента (LED мигает)
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
  digitalWrite(LED_PIN, HIGH);  // Подключились

  // Init support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_imu", "", &support));

  // Publisher: /imu/data
  RCCHECK(rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"
  ));

  // Init IMU message
  initImuMessage();

  // Executor (без подписчиков, только для spin)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

// ============================================================================
// IMU MESSAGE INITIALIZATION
// ============================================================================

void initImuMessage() {
  // Аллокация строки frame_id
  imu_msg.header.frame_id.data = (char*)malloc(20);
  imu_msg.header.frame_id.capacity = 20;
  snprintf(imu_msg.header.frame_id.data, 20, "imu_link");
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

  // Ориентация не публикуется (covariance[0] = -1)
  // "please set element 0 of the associated covariance matrix to -1"
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
  imu_msg.orientation_covariance[0] = -1.0;

  // Ковариация угловой скорости (диагональ)
  // BMG при 500DPS: noise ~0.014 deg/s = ~0.000244 rad/s -> variance ~6e-8
  // Ставим консервативно
  memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
  imu_msg.angular_velocity_covariance[0] = 0.001;  // X
  imu_msg.angular_velocity_covariance[4] = 0.001;  // Y
  imu_msg.angular_velocity_covariance[8] = 0.001;  // Z

  // Ковариация линейного ускорения (диагональ)
  // BMA при 4G: noise ~0.25 mg/sqrt(Hz) -> conservative estimate
  memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));
  imu_msg.linear_acceleration_covariance[0] = 0.01;  // X
  imu_msg.linear_acceleration_covariance[4] = 0.01;  // Y
  imu_msg.linear_acceleration_covariance[8] = 0.01;  // Z
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // Читаем датчики
  sensorA.read(BMA_M_S);    // m/s² (кажущееся ускорение, включает гравитацию)
  sensorG.read(BMG_RAD_S);  // rad/s

  // Process micro-ROS callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Публикация с интервалом
  if (now - lastPublish >= IMU_PUBLISH_MS) {
    lastPublish = now;

    // Timestamp
    imu_msg.header.stamp.sec = (int32_t)(now / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((now % 1000) * 1000000);

    // Угловая скорость (rad/s) с вычитанием bias
    imu_msg.angular_velocity.x = sensorG.axisX - gyroBiasX;
    imu_msg.angular_velocity.y = sensorG.axisY - gyroBiasY;
    imu_msg.angular_velocity.z = sensorG.axisZ - gyroBiasZ;

    // Линейное ускорение (m/s²)
    imu_msg.linear_acceleration.x = sensorA.axisX;
    imu_msg.linear_acceleration.y = sensorA.axisY;
    imu_msg.linear_acceleration.z = sensorA.axisZ;

    // Публикуем
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  }

  delay(1);
}
