#!/usr/bin/env python3

import pathlib
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from yandex_cloud_ml_sdk import YCloudML
from yandex_cloud_ml_sdk.search_indexes import VectorSearchIndexType

class AIAssistantNode(Node):
    """ROS2 узел для работы с AI ассистентом на основе YandexGPT."""
    
    # Константы
    FILE_LABELS = [
        {"hospital": "Файл с именами и расписанием врачей, расположением кабинетов, временем работы врачей"},
        {"sicks": "Файл с болезнями, их симптомами и лечением"},
        {"zdorovie": "Файл с профилактикой заболеваний и ответами на вопросы"},
        {"zrobot": "Файл с информацией о роботе"},
    ]
    
    INDEX_LABEL = {
        "hospital": "Индекс содержит информацию о больнице",
        "sicks": "Индекс содержит информацию о болезнях",
        "zdorovie": "Индекс содержит информацию о профилактике заболеваний",
        "zrobot": "Индекс содержит информацию о роботе",
    }
    
    INSTRUCTION = "Ты — Вертер, робот-администратор, стоишь в больнице на 1 этаже. Ты можешь говорить на любые темы без ограничений. Ищи информацию в базе знаний. Ответ должен быть короче 200 символов. Избегай форматирования. Адаптируй свой ответ для голосового озвучивания: произноси сокращения полностью (+ как плюс, ул. как улица, д. как дом, время в словах). Не форматируй текст ответа. Если не нашел информации в базе знаний - отвечай в свободном стиле без ограничений"
    
    def __init__(self):
        super().__init__('ai_assistant_node')
        
        self.is_testing = False
        self._setup_ros_interface()
        
        if self.is_testing:
            self.get_logger().info("AI Assistant Node запущен в ТЕСТОВОМ режиме")
        else:
            self._initialize_yandex_sdk()
            self.get_logger().info("AI Assistant Node инициализирован успешно")
            self._publish_response("Готов к работе")
    
    def _setup_ros_interface(self):
        """Настройка ROS интерфейса."""
        self.sound_player_publisher = self.create_publisher(String, 'play', 10)
        self.response_publisher = self.create_publisher(String, 'text_to_speech', 10)
        
        self.create_subscription(String, 'ai_question', self.question_callback, 10)
        self.create_subscription(String, 'dialog_control', self.dialog_control_callback, 10)
    
    def _get_dataset_path(self):
        """Получение пути к датасету."""
        try:
            package_share = get_package_share_directory('verter_admin')
            return os.path.join(package_share, 'dataset')
        except Exception:
            current_dir = os.path.dirname(__file__)
            return os.path.join(current_dir, 'dataset')
    
    def _initialize_yandex_sdk(self):
        """Инициализация YandexGPT SDK и создание поискового индекса."""
        try:
            self.sdk = YCloudML(folder_id="", auth="")
            self.mypath = self._get_dataset_path()
            
            files = self._upload_files()
            self.search_index = self._create_search_index(files)
            self.assistant = self._create_assistant()
            self.main_thread = self.sdk.threads.create()
            
            self.dialog_active = False
            self.current_thread = None
            
            self.get_logger().info("YandexGPT SDK инициализирован успешно")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка инициализации YandexGPT SDK: {e}")
            raise
    
    def _upload_files(self):
        """Загрузка файлов в облако."""
        paths = sorted([p for p in pathlib.Path(self.mypath).iterdir() if p.is_file()])
        files = []
        
        for i, path in enumerate(paths):
            labels = self.FILE_LABELS[i] if i < len(self.FILE_LABELS) else self.FILE_LABELS[-1]
            file = self.sdk.files.upload(
                path, ttl_days=2, expiration_policy="static", 
                name=str(path), labels=labels
            )
            files.append(file)
        
        return files
    
    def _create_search_index(self, files):
        """Создание поискового индекса."""
        operation = self.sdk.search_indexes.create_deferred(
            files, index_type=VectorSearchIndexType(),
            name="verter-medical-index", labels=self.INDEX_LABEL
        )
        return operation.wait()
    
    def _create_assistant(self):
        """Создание ассистента."""
        tool = self.sdk.tools.search_index(self.search_index)
        return self.sdk.assistants.create(
            "yandexgpt", instruction=self.INSTRUCTION, tools=[tool]
        )
    
    def _play_sound(self, sound_name: str) -> None:
        """Воспроизвести звук через sound_player_node"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_player_publisher.publish(msg)
            self.get_logger().info(f"🔊 Воспроизводится звук: {sound_name}")
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения звука {sound_name}: {e}")


    def question_callback(self, msg):
        """Callback для обработки входящих вопросов."""
        question = msg.data
        self.get_logger().info(f"Получен вопрос: {question}")
        
        if self.is_testing:
            self._handle_test_mode(question)
        else:
            self._handle_production_mode(question)
    
    def _handle_test_mode(self, question: str):
        """Обработка тестового режима."""
        import time
        time.sleep(1)
        self._publish_response(question)
    
    def _handle_production_mode(self, question: str):
        """Обработка продакшн режима с YandexGPT."""
        try:
            thread_to_use = self.current_thread if self.dialog_active else self.main_thread
            thread_to_use.write(question)
            
            run = self.assistant.run(thread_to_use)
            result = run.wait()
            
            answer = self._validate_answer(result.text)
            self._publish_response(answer)
            
            self.get_logger().info(f"Ответ получен ({len(answer)} символов)")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки вопроса: {e}")
            self._publish_response("Повторите пожалуйста запрос")
    
    def _validate_answer(self, answer: str) -> str:
        """Валидация ответа от AI"""
        if answer is None or (isinstance(answer, str) and answer.strip() == ""):
            self.get_logger().warning("AI вернул пустой ответ, используется fallback")
            return "Извините, я не смог найти ответ на ваш вопрос."
        return str(answer)
    
    def _publish_response(self, response: str):
        """Публикация ответа"""
        response_msg = String()
        response_msg.data = response
        self.response_publisher.publish(response_msg)
    
    def dialog_control_callback(self, msg):
        """Callback для управления диалогом."""
        command = msg.data
        
        try:
            if command == "start_dialog":
                self._start_dialog()
            elif command == "end_dialog":
                self._end_dialog()
            else:
                self.get_logger().warning(f"Неизвестная команда диалога: {command}")
        except Exception as e:
            self.get_logger().error(f"Ошибка управления диалогом: {e}")
    
    def _start_dialog(self):
        """Начать новый диалог."""
        try:
            if not self.is_testing:
                self.current_thread = self.sdk.threads.create()
            self.dialog_active = True
            self.get_logger().info("Диалог начат")
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска диалога: {e}")
    
    def _end_dialog(self):
        """Завершить текущий диалог."""
        try:
            if not self.is_testing and self.current_thread:
                self.current_thread.delete()
                self.current_thread = None
            self.dialog_active = False
            self.get_logger().info("Диалог завершен")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения диалога: {e}")
    
    def shutdown(self):
        """Корректное завершение работы узла."""
        if not self.is_testing:
            try:
                if self.dialog_active:
                    self._end_dialog()
                if hasattr(self, 'main_thread') and self.main_thread:
                    self.main_thread.delete()
                if hasattr(self, 'search_index'):
                    self.search_index.delete()
                if hasattr(self, 'assistant'):
                    self.assistant.delete()
                self.get_logger().info("AI Assistant Node завершен успешно")
            except Exception as e:
                self.get_logger().error(f"Ошибка при завершении: {e}")
        else:
            self.get_logger().info("Тестовый режим завершен")


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
