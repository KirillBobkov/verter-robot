import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
import curses
import time
import threading


class SSHKeyTeleop(Node):
    def __init__(self):
        super().__init__('teleop_ssh_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Настройки шага скорости
        self.lin_step = 0.01
        self.ang_step = 0.05  
        
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Мьютекс для безопасного изменения скоростей между потоками
        self.lock = threading.Lock()
        
        self.is_run_action = False
        self.is_stop = False

        # Создаем ROS2 таймер на 20 Гц (0.05 секунды)
        #self.timer = self.create_timer(0.05, self.timer_callback)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        with self.lock:
            if self.is_run_action:
                # Публикуем скорость с фиксированной частотой
                msg = Twist()
                msg.linear.x = self.target_linear
                msg.angular.z = self.target_angular
                self.publisher_.publish(msg)
            
            if self.is_stop:
                self.is_run_action = False
                self.is_stop = False


def keyboard_listener(stdscr, node: Node):
    # Оставляем curses БЛОКИРУЮЩИМ (nodelay НЕ вызываем)
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.refresh()

    stdscr.addstr(0, 0, "=== SSH ROS2 TELEOP (TIMERS EDITION) ===")
    stdscr.addstr(1, 0, "Используйте СТРЕЛКИ для управления.")
    stdscr.addstr(2, 0, "ПРОБЕЛ — экстренный стоп. 'Q' — выход.")
    stdscr.refresh()

    try:
        while rclpy.ok():
            # Программа спит здесь и НЕ ест процессор, пока кнопка не нажата
            key = stdscr.get_wch()

            with node.lock:
                match key:
                    case curses.KEY_UP:
                        node.target_linear += node.lin_step
                    case curses.KEY_DOWN:
                        node.target_linear -= node.lin_step
                    case curses.KEY_LEFT:
                        node.target_angular += node.ang_step
                    case curses.KEY_RIGHT:
                        node.target_angular -= node.ang_step
                    case _ if key == ' ':
                        node.target_linear = 0.0
                        node.target_angular = 0.0
                    case _ if  key in ['q', 'Q', 'й', 'Й']:
                        break
                
                node.target_linear = round(node.target_linear, 2)
                node.target_angular = round(node.target_angular, 2)
            
            if node.target_linear == 0.0 and node.target_angular == 0.0:
                node.is_stop = True
            else:
                node.is_run_action = True

            if node.target_linear > 0.0:
                dir = 'Едет вперёд'
            elif node.target_linear < 0.0:
                dir = 'Едет назад'
            else:
                dir = 'Стоит на месте'

            if node.target_angular > 0.0:
                dir += ' и поворачивает влево          '
            elif node.target_angular < 0.0:
                dir += ' и поворачивает вправо          '
            else:
                dir += ' и не поворачивает          '

            # Обновляем интерфейс
            stdscr.addstr(4, 0, f"Линейная (X): {node.target_linear:0.2f} м/с   ")
            stdscr.addstr(5, 0, f"Угловая  (Z): {node.target_angular:0.2f} рад/с  ")
            stdscr.addstr(7, 0, f"Состояние: {dir}")
            stdscr.refresh()

    finally:
        curses.nocbreak()
        stdscr.keypad(False)
        curses.endwin()
        rclpy.shutdown() # Останавливаем ROS2 при выходе из curses

def main(args=None):
    rclpy.init(args=args)
    node = SSHKeyTeleop()

    # Важно: используем MultiThreadedExecutor, чтобы таймер работал параллельно с curses
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Запускаем ROS2 executor в отдельном фоновом потоке
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Основной поток отдаем под curses интерфейс
    curses.wrapper(keyboard_listener, node)

if __name__ == '__main__':
    main()
