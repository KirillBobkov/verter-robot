#!/usr/bin/env python3
"""
DOA Node - отдельная нода для обработки Direction of Arrival (DOA)
Управляет поворотом головы робота на основе направления звука
"""

import threading
import time
import subprocess
from typing import Optional, List

import rclpy
from rclpy.node import Node
import usb.core
import usb.util
import serial
import serial.tools.list_ports
from std_msgs.msg import String, Bool

from .tuning import Tuning

# Константы
RESPEAKER_VENDOR_ID = 0x2886
RESPEAKER_PRODUCT_ID = 0x0018
DOA_POLLING_INTERVAL = 7.0

# Константы для Arduino подключения
ARDUINO_DEVPATH = "1.3"  # devpath для Arduino головы
BAUD_RATE = 9600
CONNECTION_TIMEOUT = 2.0
SERIAL_TIMEOUT = 0.1

# DOA углы для команд головы
DOA_CENTER_MIN = 10
DOA_CENTER_MAX = 150
DOA_RIGHT_MIN = 270
DOA_RIGHT_MAX = 360
DOA_RIGHT_EDGE_MAX = 10
DOA_LEFT_MIN = 150
DOA_LEFT_MAX = 270

# Команды головы
HEAD_CENTER = "HEAD:CENTER"
HEAD_RIGHT = "HEAD:RIGHT"
HEAD_LEFT = "HEAD:LEFT"


class DOANode(Node):
    """Нода для обработки Direction of Arrival (DOA)"""
    
    def __init__(self):
        super().__init__('doa_node')
        
        # Переменные состояния
        self.shutdown_event = threading.Event()
        self.last_eye_command: Optional[str] = None
        self.doa_enabled = False
        
        # Arduino подключение
        self.arduino_serial: Optional[serial.Serial] = None
        self.current_port = None
        
        # Настройка ROS2 коммуникации
        self._setup_ros_communication()
        
        # Подключение к Arduino
        if self._connect_to_arduino():
            self.get_logger().info("✓ Arduino для головы подключен")
        else:
            self.get_logger().warn("⚠️ Arduino для головы не подключен")
        
        # Инициализация DOA
        self._setup_doa()
        
        # Запуск DOA потока
        self._start_doa_thread()
        
        self.get_logger().info("🎯 DOA Node запущен")

    def _setup_ros_communication(self):
        """Настройка ROS2 subscribers"""
        # Subscriber для команд головы из других нод
        self.create_subscription(
            String,
            'head_commands',
            self._handle_head_command,
            10
        )
        self.get_logger().info('Подписчик для head_commands создан')
        
        # Subscriber для управления активностью DOA
        self.create_subscription(
            Bool,
            'doa_active',
            self._handle_doa_control,
            10
        )

    def _find_arduino_port(self) -> List[str]:
        """Автоматический поиск портов Arduino по devpath."""
        available_ports = []
        
        self.get_logger().info(f'Поиск устройства с devpath={ARDUINO_DEVPATH} для головы')
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            self.get_logger().info(f'Проверяю порт: {port.device}')
            
            # Проверяем devpath с помощью udevadm
            try:
                result = subprocess.run(
                    ['udevadm', 'info', '-a', '-n', port.device], 
                    capture_output=True, 
                    text=True
                )
                
                if f'devpath=="{ARDUINO_DEVPATH}"' in result.stdout or f'ATTRS{{devpath}}=="{ARDUINO_DEVPATH}"' in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath={ARDUINO_DEVPATH} на порту {port.device}')
                    available_ports.append(port.device)
                else:
                    self.get_logger().info(f'Порт {port.device} не соответствует devpath={ARDUINO_DEVPATH}')
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')
        
        if not available_ports:
            self.get_logger().warn(f'Не найдено устройств с devpath={ARDUINO_DEVPATH}')
        
        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino для головы."""
        available_ports = self._find_arduino_port()
        
        if not available_ports:
            self.get_logger().error(f'Не найдено устройств с devpath={ARDUINO_DEVPATH} для головы')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для головы (devpath={ARDUINO_DEVPATH})')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                BAUD_RATE, 
                timeout=SERIAL_TIMEOUT
            )
            self.current_port = port
            
            time.sleep(CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Arduino головы на порту {port} (devpath={ARDUINO_DEVPATH})')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Arduino головы на порту {port}: {e}')
            return False

    def _is_arduino_connected(self) -> bool:
        """Проверка подключения Arduino."""
        return self.arduino_serial and self.arduino_serial.is_open

    def _send_to_arduino(self, command: str):
        """Отправка команды на Arduino."""
        if not self._is_arduino_connected():
            self.get_logger().warn('Arduino для головы не подключен')
            return
            
        try:
            self.arduino_serial.write(f"{command}\n".encode('utf-8'))
            self.get_logger().debug(f'Команда отправлена на Arduino: "{command}"')
        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка отправки на Arduino: {e}')
            # Попытка переподключения
            self._reconnect_arduino()
        except Exception as e:
            self.get_logger().error(f'Неожиданная ошибка отправки на Arduino: {e}')

    def _reconnect_arduino(self):
        """Переподключение к Arduino."""
        self.get_logger().info('Попытка переподключения к Arduino головы...')
        self._close_arduino_connection()
        
        if self._connect_to_arduino():
            self.get_logger().info('Успешно переподключен к Arduino головы')
        else:
            self.get_logger().warn('Не удалось переподключиться к Arduino головы')

    def _close_arduino_connection(self):
        """Закрытие соединения с Arduino."""
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()

    def _handle_head_command(self, msg: String):
        """Обработчик команд головы из топика head_commands."""
        command = msg.data
        self.get_logger().info(f'Получена команда головы: "{command}"')
        self._send_to_arduino(command)

    def _setup_doa(self):
        """Инициализация DOA"""
        try:
            self.doa_dev = usb.core.find(idVendor=RESPEAKER_VENDOR_ID, idProduct=RESPEAKER_PRODUCT_ID)
            if self.doa_dev is None:
                self.doa_enabled = False
                self.get_logger().warn("⚠️ ReSpeaker устройство не найдено - DOA отключен")
            else:
                self.mic_tuning = Tuning(self.doa_dev)
                self.doa_enabled = True
                self.get_logger().info("✓ DOA инициализирован")
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка инициализации DOA: {e}")
            self.doa_enabled = False

    def _start_doa_thread(self):
        """Запуск отдельного треда для опроса DOA"""
        self.doa_thread = threading.Thread(target=self._doa_loop, daemon=True)
        self.doa_thread.start()
        self.get_logger().info(f"✓ DOA поток запущен (интервал: {DOA_POLLING_INTERVAL}с)")

    def _doa_loop(self):
        """Периодический опрос DOA"""
        while not self.shutdown_event.is_set():
            try:
                if self.doa_enabled:
                    self._handle_doa()
            except Exception as e:
                self.get_logger().error(f"Ошибка DOA loop: {e}")
            
            # Ожидаем или выходим, если ноду попросили завершиться
            if self.shutdown_event.wait(DOA_POLLING_INTERVAL):
                break

    def _handle_doa(self):
        """Обработка DOA - получение угла и отправка команды голове"""
        try:
            doa_angle = self.mic_tuning.direction
            self.get_logger().info(f"DOA угол: {doa_angle}")
            if doa_angle is not None:
                self._send_eye_command_by_doa(doa_angle)
        except Exception as e:
            self.get_logger().error(f"Ошибка получения DOA: {e}")

    def _send_eye_command_by_doa(self, doa_angle: int):
        """Отправить команду голове на основе DOA угла"""
        try:
            command = self._convert_doa_angle_to_command(doa_angle)
            if not command:
                self.get_logger().debug(f"Некорректный DOA угол: {doa_angle}°")
                return
            
            # Debounce: пропускаем повторяющиеся команды подряд
            if command == self.last_eye_command:
                self.get_logger().debug(f"⚠️ Пропускаем дублирующую DOA команду {command}")
                return
            self.last_eye_command = command

            # Отправляем команду напрямую на Arduino
            self._send_to_arduino(command)
            self.get_logger().debug(f"🎯 DOA команда отправлена: {command} (угол: {doa_angle}°)")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки команды голове: {e}")

    def _convert_doa_angle_to_command(self, doa_angle: int) -> Optional[str]:
        """Конвертация DOA угла в команду"""
        if DOA_CENTER_MIN <= doa_angle <= DOA_CENTER_MAX:
            return HEAD_CENTER
        elif (DOA_RIGHT_MIN <= doa_angle <= DOA_RIGHT_MAX) or (0 <= doa_angle < DOA_RIGHT_EDGE_MAX):
            return HEAD_RIGHT
        elif DOA_LEFT_MIN < doa_angle < DOA_LEFT_MAX:
            return HEAD_LEFT
        return None

    def _handle_doa_control(self, msg: Bool):
        """Управление активностью DOA"""
        try:
            if msg.data:
                self.get_logger().info("🎯 DOA активирован")
            else:
                self.get_logger().info("🎯 DOA деактивирован")
                # Сбрасываем последнюю команду при деактивации
                self.last_eye_command = None
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка управления DOA: {e}")

    def _cleanup_resources(self):
        """Безопасная очистка всех ресурсов при завершении"""
        # Закрываем Arduino соединение
        self._close_arduino_connection()
        
        # Освобождаем USB ресурсы
        if hasattr(self, 'doa_dev') and self.doa_dev:
            try:
                usb.util.dispose_resources(self.doa_dev)
                self.get_logger().info("✓ USB ресурсы освобождены")
            except Exception as e:
                self.get_logger().warn(f"Ошибка освобождения USB: {e}")

    def shutdown(self):
        """Завершение работы с очисткой ресурсов"""
        try:
            self.get_logger().info("🔄 Завершение работы DOA ноды...")
            
            # Устанавливаем событие завершения
            self.shutdown_event.set()
            
            # Безопасно очищаем ресурсы
            self._cleanup_resources()
            
            self.get_logger().info("✓ Завершение работы DOA ноды завершено")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка при завершении DOA ноды: {e}")


def main(args=None):
    """Главная функция"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = DOANode()
        print("🎯 DOA NODE: Direction of Arrival для управления головой")
        print("   Команды: HEAD:CENTER, HEAD:LEFT, HEAD:RIGHT")
        print("   Управление через топик: doa_active")
        print("   Команды головы через топик: head_commands")
        print(f"   Arduino подключение: devpath={ARDUINO_DEVPATH}")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🔄 Завершение DOA ноды по Ctrl+C...")
    except Exception as e:
        print(f"\n❌ Критическая ошибка DOA ноды: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✓ DOA нода завершена")


if __name__ == '__main__':
    main()
