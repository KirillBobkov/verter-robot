#!/usr/bin/env python3

import pathlib
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from yandex_cloud_ml_sdk import YCloudML
from yandex_cloud_ml_sdk.search_indexes import (
    StaticIndexChunkingStrategy,
    TextSearchIndexType,
)

class AIAssistantNode(Node):
    """ROS2 узел для работы с AI ассистентом на основе YandexGPT."""
    
    def __init__(self):
        super().__init__('ai_assistant_node')
        
        # Параметры для YandexGPT
        try:
            package_share = get_package_share_directory('verter_admin')
            self.mypath = os.path.join(package_share, 'dataset')
        except Exception as e:
            self.get_logger().error(f"Не удалось найти пакет verter_admin: {e}")
            # Fallback путь для разработки
        self.mypath = "/mnt/c/Users/Пользователь/Documents/verter-robot/verter_admin/src/verter_admin/ai_assistant/dataset/"
        self.folder = "вставить"
        self.token = "вставить"
        self.instruction = "Выполняй поиск по базе знаний и не выдумывай ответ"
        
        # Инициализация SDK и индекса
        self._initialize_yandex_sdk()
        
        # Создание subscriber для получения вопросов
        self.subscription = self.create_subscription(
            String,
            'ai_question',
            self.question_callback,
            10
        )
        
        # Создание publisher для отправки ответов
        self.response_publisher = self.create_publisher(String, 'ai_response', 10)
        
        self.get_logger().info("AI Assistant Node инициализирован успешно")
    
    def _initialize_yandex_sdk(self):
        """Инициализация YandexGPT SDK и создание поискового индекса."""
        try:
            self.sdk = YCloudML(
                folder_id=self.folder,
                auth=self.token,
            )
            
            # Загружаем файлы
            paths = pathlib.Path(self.mypath).iterdir()
            files = []
            for path in paths:
                if path.is_file():
                    file = self.sdk.files.upload(
                        path,
                        ttl_days=5,
                        expiration_policy="static",
                    )
                    files.append(file)
            
            self.files = files
            
            # Создаем поисковый индекс
            operation = self.sdk.search_indexes.create_deferred(
                files,
                index_type=TextSearchIndexType(
                    chunking_strategy=StaticIndexChunkingStrategy(
                        max_chunk_size_tokens=700,
                        chunk_overlap_tokens=300,
                    )
                ),
            )
            
            # Ожидаем создания индекса
            self.search_index = operation.wait()
            
            # Создаем инструмент поиска
            tool = self.sdk.tools.search_index(
                self.search_index,
                call_strategy={
                    "type": "function",
                    "function": {"name": "guide", "instruction": self.instruction},
                },
            )
            
            # Создаем ассистента
            self.assistant = self.sdk.assistants.create(
                "yandexgpt",
                instruction="Ты — робот-администратор который стоит в поликлиннике. Отвечай вежливо. Если информация не содержится в документах ниже, не придумывай ответ. В ответе озвучивай все цифры буквами (например: 8:00 - восемь ноль ноль)",
                tools=[tool],
            )
            
            self.thread = self.sdk.threads.create()
            
            self.get_logger().info("YandexGPT SDK инициализирован успешно")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка инициализации YandexGPT SDK: {e}")
            raise
    
    def question_callback(self, msg):
        """Callback для обработки входящих вопросов."""
        question = msg.data
        self.get_logger().info(f"Получен вопрос: {question}")
        
        try:
            # Создаем новый thread для каждого вопроса (без истории)
            thread = self.sdk.threads.create()
            
            # Отправляем только новый вопрос
            thread.write(question)
            run = self.assistant.run(thread)
            result = run.wait()
            
            # Выводим ответ в консоль
            answer = result.text
            self.get_logger().info(f"\nОтвет AI: \n{answer}")
            
            # Публикуем ответ в топик ai_response
            response_msg = String()
            response_msg.data = answer
            self.response_publisher.publish(response_msg)
            self.get_logger().info("Ответ опубликован в топик ai_response")
            
            # Удаляем thread после использования для освобождения ресурсов
            thread.delete()
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки вопроса: {e}")
    
    def shutdown(self):
        """Корректное завершение работы узла."""
        try:
            if hasattr(self, 'search_index'):
                self.search_index.delete()
            if hasattr(self, 'assistant'):
                self.assistant.delete()
            if hasattr(self, 'files'):
                for file in self.files:
                    file.delete()
            self.get_logger().info("AI Assistant Node завершен успешно")
        except Exception as e:
            self.get_logger().error(f"Ошибка при завершении: {e}")


def main(args=None):
    """Основная функция для запуска узла AI ассистента."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = AIAssistantNode()
        print("AI Assistant Node запущен. Ожидание вопросов...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nЗавершение работы...")
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
