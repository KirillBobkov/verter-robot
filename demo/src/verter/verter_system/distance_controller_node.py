import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class DistanceControllerNode(Node):
    def __init__(self):
        super().__init__('distance_controller_node')
        
        # Создаем подписчика на топик /distance_h04
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/distance_h04',
            self.distance_callback,
            10
        )
        self.get_logger().info('Подписчик для /distance_h04 создан.')
        
        # Создаем издателя для топика /verter_commands
        self.command_publisher = self.create_publisher(
            String, 
            '/verter_commands', 
            10
        )
        self.get_logger().info('Издатель для /verter_commands создан.')
        
        # Создаем издателя для топика /verter_say
        self.say_publisher = self.create_publisher(
            String, 
            '/verter_say', 
            10
        )
        self.get_logger().info('Издатель для /verter_say создан.')
        
        # Пороговое значение расстояния (в см)
        # Можно настроить через параметр
        self.declare_parameter('distance_threshold', 20.0)
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.get_logger().info(f'Установлен порог расстояния: {self.distance_threshold} см')
        
        self.get_logger().info('Узел датчика расстояния инициализирован.')

    def distance_callback(self, msg):
        """Обработчик сообщений с датчика расстояния."""
        distance = msg.data
        self.get_logger().info(f'Получено расстояние: {distance:.2f} см')
        
        # Упрощенная логика: если расстояние меньше порога - стоп и звуковой сигнал
        command_msg = String()
        
        if distance > self.distance_threshold:
            # Если расстояние больше порога - путь свободен
            self.get_logger().info(f'Путь свободен. Расстояние: {distance:.2f} см (порог: {self.distance_threshold} см)')
        else:
            # Если расстояние меньше порога - препятствие, останавливаемся и воспроизводим звук
            command_msg.data = "CHASSIS:STOP"
            self.command_publisher.publish(command_msg)
            self.get_logger().info(f'Отправлена команда в /verter_commands: "{command_msg.data}"')
            
            # Воспроизводим звук о препятствии
            say_msg = String()
            say_msg.data = "obstacle"
            self.say_publisher.publish(say_msg)
            self.get_logger().info(f'Отправлена команда в /verter_say: "{say_msg.data}"')
            self.get_logger().info(f'Обнаружено препятствие на расстоянии {distance:.2f} см (порог: {self.distance_threshold} см)')
    
    def destroy_node(self):
        """Корректное завершение работы узла."""
        self.get_logger().info('Завершение работы узла датчика расстояния.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    distance_controller_node = DistanceControllerNode()
    
    try:
        rclpy.spin(distance_controller_node)
    except KeyboardInterrupt:
        distance_controller_node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        distance_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 