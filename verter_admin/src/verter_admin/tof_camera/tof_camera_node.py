#!/usr/bin/env python3
"""
Нода для Arducam TOF камеры US981.
Захватывает данные глубины, конвертирует в PointCloud2 и публикует для Nav2.
"""

import rclpy
from rclpy.node import Node
import ArducamDepthCamera as ac
import numpy as np
import threading
import time
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class TOFCameraNode(Node):
    """Нода для работы с Arducam TOF камерой через CSI."""
    
    def __init__(self):
        super().__init__('tof_camera_node')

        # Параметры
        self.declare_parameter('fps', 15)
        self.declare_parameter('frame_id', 'camera_tof_optical')
        self.declare_parameter('depth_min', 0.1)  # метры
        self.declare_parameter('depth_max', 5.0)  # метры
        self.declare_parameter('fov_h', 60.0)     # горизонтальный FOV в градусах
        self.declare_parameter('fov_v', 45.0)     # вертикальный FOV в градусах
        self.declare_parameter('downsample', 2)    # субдискретизация (каждый N-й пиксель)

        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.depth_min = self.get_parameter('depth_min').value
        self.depth_max = self.get_parameter('depth_max').value
        self.fov_h = self.get_parameter('fov_h').value
        self.fov_v = self.get_parameter('fov_v').value
        self.downsample = self.get_parameter('downsample').value

        # Publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            'camera/depth/points',
            10
        )

        # Статистика
        self.frame_count = 0
        self.error_count = 0
        self.last_stats_time = time.time()
        self.stop_capture = False

        # Arducam камера
        self.cam = None
        self.capture_thread = None
        
        # Инициализация
        if self._initialize_camera():
            self.get_logger().info('TOF Camera инициализирована успешно')
            self._start_capture()
        else:
            self.get_logger().error('Не удалось инициализировать TOF камеру')

    def _initialize_camera(self):
        """Инициализация Arducam TOF камеры."""
        try:
            self.get_logger().info('Открываем Arducam TOF камеру...')
            self.cam = ac.ArducamCamera()
            
            # Открываем CSI камеру
            ret = self.cam.open(ac.Connection.CSI, 0)
            if ret != 0:
                self.get_logger().error(f'Ошибка открытия камеры: {ret}')
                return False
            
            # Запускаем захват RAW данных
            ret = self.cam.start(ac.FrameType.RAW)
            if ret != 0:
                self.get_logger().error(f'Ошибка запуска камеры: {ret}')
                self.cam.close()
                return False
            
            self.get_logger().info(f'Камера запущена. SDK: {ac.__version__}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Исключение при инициализации: {e}')
            return False

    def _start_capture(self):
        """Запуск потока захвата."""
        self.stop_capture = False
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        self.get_logger().info('Поток захвата запущен')

    def _publish_pointcloud(self, depth_image):
        """
        Преобразует depth image в PointCloud2 и публикует.

        Args:
            depth_image: numpy array (H x W) с данными глубины в мм
        """
        try:
            # Получаем размеры изображения
            height, width = depth_image.shape

            # Вычисляем фокальные расстояния на основе FOV
            fx = width / (2.0 * np.tan(np.radians(self.fov_h) / 2.0))
            fy = height / (2.0 * np.tan(np.radians(self.fov_v) / 2.0))
            cx = width / 2.0
            cy = height / 2.0

            # Создаем массив точек
            points = []

            for v in range(0, height, self.downsample):
                for u in range(0, width, self.downsample):
                    # Глубина в метрах (данные в мм)
                    z = depth_image[v, u] / 1000.0

                    # Фильтруем по диапазону
                    if z < self.depth_min or z > self.depth_max:
                        continue

                    # Фильтруем нулевые/невалидные значения
                    if z == 0 or np.isnan(z) or np.isinf(z):
                        continue

                    # Проецируем в 3D (камера координаты)
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    points.append([x, y, z])

            if len(points) == 0:
                return

            # Создаем PointCloud2 сообщение
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            # Упаковываем данные
            cloud_data = []
            for point in points:
                cloud_data.append(struct.pack('fff', *point))

            pointcloud_msg = PointCloud2(
                header=header,
                height=1,
                width=len(points),
                is_dense=False,
                is_bigendian=False,
                fields=fields,
                point_step=12,  # 3 floats * 4 bytes
                row_step=12 * len(points),
                data=b''.join(cloud_data)
            )

            self.pointcloud_pub.publish(pointcloud_msg)

        except Exception as e:
            self.get_logger().error(f'Ошибка при публикации PointCloud: {e}')

    def _capture_loop(self):
        """Основной цикл захвата кадров."""
        while not self.stop_capture:
            try:
                # Запрашиваем кадр с таймаутом 2000ms
                frame = self.cam.requestFrame(2000)

                if frame is not None and isinstance(frame, ac.RawData):
                    self.frame_count += 1
                    raw_data = frame.raw_data
                    self.cam.releaseFrame(frame)

                    if raw_data is not None and len(raw_data) > 0:
                        # Публикуем PointCloud2 для Nav2
                        self._publish_pointcloud(raw_data)

                        # Логируем данные
                        self._log_frame_data(raw_data)
                else:
                    self.error_count += 1
                    if self.error_count % 10 == 0:
                        self.get_logger().warn(f'Ошибок захвата: {self.error_count}')

                # Публикация статистики каждые 5 секунд
                current_time = time.time()
                if current_time - self.last_stats_time >= 5.0:
                    self._log_stats()
                    self.last_stats_time = current_time

                # Контроль FPS
                time.sleep(1.0 / self.fps)

            except Exception as e:
                self.get_logger().error(f'Ошибка в цикле захвата: {e}')
                time.sleep(1.0)

    def _log_frame_data(self, raw_data):
        """Логирование данных кадра."""
        if self.frame_count % 30 == 0:  # Каждые 30 кадров
            min_val = np.min(raw_data)
            max_val = np.max(raw_data)
            mean_val = np.mean(raw_data)
            self.get_logger().info(
                f'Кадр #{self.frame_count}: '
                f'min={min_val:.0f}, max={max_val:.0f}, mean={mean_val:.0f}, '
                f'shape={raw_data.shape}'
            )

    def _log_stats(self):
        """Логирование статистики."""
        fps = self.frame_count / 5.0
        self.get_logger().info(
            f'TOF Camera: кадров={self.frame_count}, '
            f'ошибок={self.error_count}, '
            f'FPS={fps:.1f}'
        )
        # Сброс счетчика
        self.frame_count = 0

    def destroy_node(self):
        """Корректное завершение работы."""
        self.get_logger().info('Остановка TOF Camera ноды')
        self.stop_capture = True
        
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        if self.cam:
            self.cam.stop()
            self.cam.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TOFCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прерывание (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
