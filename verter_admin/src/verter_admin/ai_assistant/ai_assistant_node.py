#!/usr/bin/env python3

import pathlib
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

from yandex_cloud_ml_sdk import YCloudML
from yandex_cloud_ml_sdk.search_indexes import (
    StaticIndexChunkingStrategy,
    TextSearchIndexType,
    VectorSearchIndexType,
)

class AIAssistantNode(Node):
    """ROS2 узел для работы с AI ассистентом на основе YandexGPT."""
    
    def __init__(self):
        super().__init__('ai_assistant_node')
        
        # Параметр тестового режима
        self.isTesting = False

                # Publisher для воспроизведения звуков
        self.sound_player_publisher = self.create_publisher(String, '/play', 10)

        # Создание subscriber для получения вопросов
        self.subscription = self.create_subscription(
            String,
            'ai_question',
            self.question_callback,
            10
        )
        
        # Создание publisher для отправки ответов
        self.response_publisher = self.create_publisher(String, 'ai_response', 10)
        
        # Subscriber для управления диалогом
        self.create_subscription(
            String,
            'dialog_control',
            self.dialog_control_callback,
            10
        )

        
        # Переменная file_labels содержит метаданные,
        # которые будут присвоены файлам-источникам.
        self.file_labels = [
            {"hospital": "Файл с расписанием врачей, расположением кабинетов, временем работы"},
            {"sicks": "Файл с болезнями, их симптомами и лечением"},
            {"zdorovie": "Файл с профилактикой заболеваний"},
            {"zrobot": "Файл с информацией о роботе"},
        ]

        self.index_label = {
            "hospital": "Индекс содержит информацию о больнице",
            "sicks": "Индекс содержит информацию о болезнях",
            "zdorovie": "Индекс содержит информацию о профилактике заболеваний",
            "zrobot": "Индекс содержит информацию о роботе",
        }

        
        if self.isTesting:
            self.get_logger().info("AI Assistant Node запущен в ТЕСТОВОМ режиме")
        else:
            # Параметры для YandexGPT
            try:
                package_share = get_package_share_directory('verter_admin')
                self.mypath = os.path.join(package_share, 'dataset')
            except Exception as e:
                self.get_logger().warning(f"Не удалось найти пакет verter_admin: {e}")
                # Fallback путь для разработки
                current_dir = os.path.dirname(__file__)
                self.mypath = os.path.join(current_dir, 'dataset')
            
            self.folder = ""
            self.token = ""
            self.instruction = "Выполняй поиск по базе знаний"
            
            # Инициализация SDK и индекса
            self._initialize_yandex_sdk()
            self.get_logger().info("AI Assistant Node инициализирован успешно")
            self._publish_response("Готов к работе")
            

    
    def _initialize_yandex_sdk(self):
        """Инициализация YandexGPT SDK и создание поискового индекса."""
        try:
            self.sdk = YCloudML(
                folder_id=self.folder,
                auth=self.token,
            )
            
            # Загружаем файлы
            paths = list(pathlib.Path(self.mypath).iterdir())
            files = []
            
            # Фильтруем только файлы и сортируем для предсказуемости
            file_paths = sorted([p for p in paths if p.is_file()])
            
            for i, path in enumerate(file_paths):
                if i < len(self.file_labels):
                    file = self.sdk.files.upload(
                        path,
                        ttl_days=2,
                        expiration_policy="static",
                        name=str(path),
                        labels=self.file_labels[i],
                    )
                    files.append(file)
                else:
                    # Если файлов больше чем лейблов, используем последний лейбл
                    file = self.sdk.files.upload(
                        path,
                        ttl_days=2,
                        expiration_policy="static",
                        name=str(path),
                        labels=self.file_labels[-1],
                    )
                    files.append(file)
            
            self.files = files
            
            # Создаем поисковый индекс
            operation = self.sdk.search_indexes.create_deferred(
                files,
                index_type=VectorSearchIndexType(),
                name="verter-medical-index",
                labels=self.index_label, 
            )
            
            # Ожидаем создания индекса
            self.search_index = operation.wait()
            
            # Создаем инструмент поиска
            tool = self.sdk.tools.search_index(self.search_index)
            
            # Создаем ассистента
            self.assistant = self.sdk.assistants.create(
                "yandexgpt",
                instruction="Ты — Вертер, робот-администратор, стоишь в больнице на 1 этаже. Ты можешь говорить на любые темы без ограничений. Ищи информацию в базе знаний. Ответ должен быть короче 200 символов. Избегай форматирования. Адаптируй свой ответ для голосового озвучивания: произноси сокращения полностью (+ как плюс, ул. как улица, д. как дом, время в словах). Если не нашел информации в базе знаний - отвечай в свободном стиле без ограничений",
                tools=[tool],
            )
            
            # Создаем основной thread для всех диалогов
            self.main_thread = self.sdk.threads.create()
            
            # Состояние диалога
            self.dialog_active = False
            self.current_thread = None
            
            self.get_logger().info("YandexGPT SDK инициализирован успешно")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка инициализации YandexGPT SDK: {e}")
            raise
    
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
        
        if self.isTesting:
            self._handle_test_mode(question)
        else:
            self._handle_production_mode(question)
    
    def _handle_test_mode(self, question: str):
        """Обработка тестового режима"""
        import time
        self.get_logger().info("Тестовый режим: перенаправляем вопрос как ответ с задержкой 1с")
        time.sleep(1)
        self._publish_response(question)
        self.get_logger().info("Вопрос перенаправлен в топик ai_response")
    
    def _handle_production_mode(self, question: str):
        """Обработка продакшн режима с YandexGPT"""
        try:
            thread_to_use = self.current_thread if self.dialog_active else self.main_thread
            
            # ЛОГИРОВАНИЕ: Что отправляем
            self.get_logger().info(f"📤 Отправка в YandexGPT:")
            self.get_logger().info(f"   Вопрос: '{question}' (длина: {len(question)} символов)")
            self.get_logger().info(f"   Thread ID: {getattr(thread_to_use, 'id', 'unknown')}")
            self.get_logger().info(f"   Dialog active: {self.dialog_active}")
            
            # Записываем вопрос в thread
            thread_to_use.write(question)
            
            # ЛОГИРОВАНИЕ: История thread
            try:
                # Пытаемся получить сообщения из thread
                if hasattr(thread_to_use, 'messages'):
                    messages = list(thread_to_use.messages)
                    self.get_logger().info(f"📋 История thread: {len(messages)} сообщений")
                    
                    # Показываем последние 3 сообщения для контроля
                    for i, msg in enumerate(messages[-3:]):
                        if hasattr(msg, 'role') and hasattr(msg, 'content'):
                            content_preview = str(msg.content)[:80] + "..." if len(str(msg.content)) > 80 else str(msg.content)
                            self.get_logger().info(f"   [{len(messages)-3+i}] {msg.role}: {content_preview}")
            except Exception as e:
                self.get_logger().debug(f"Не удалось получить историю thread: {e}")
            
            # Запускаем assistant
            self.get_logger().info("🚀 Запуск assistant.run()...")
            run = self.assistant.run(thread_to_use)
            result = run.wait()
            
            answer = self._validate_answer(result.text)
            self._publish_response(answer)
            
            dialog_info = "(диалог)" if self.dialog_active else "(одиночный)"
            self.get_logger().info(f"\n📥 Ответ AI {dialog_info} получен ({len(answer)} символов): \n{answer}")
            
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
            if not self.isTesting:
                # Создаем новый thread для диалога
                self.current_thread = self.sdk.threads.create()
                self.dialog_active = True
                self.get_logger().info("🗣️ Новый диалог начат с YandexGPT")
            else:
                self.dialog_active = True
                self.get_logger().info("🗣️ Диалог начат (тестовый режим)")
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска диалога: {e}")
    
    def _end_dialog(self):
        """Завершить текущий диалог."""
        try:
            if not self.isTesting and self.current_thread:
                # Удаляем thread диалога
                self.current_thread.delete()
                self.current_thread = None
                
            self.dialog_active = False
            self.get_logger().info("🔚 Диалог завершен")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения диалога: {e}")
    
    def shutdown(self):
        """Корректное завершение работы узла."""
        if not self.isTesting:
            try:
                # Завершаем активный диалог
                if self.dialog_active:
                    self._end_dialog()
                
                # Удаляем основной thread
                if hasattr(self, 'main_thread') and self.main_thread:
                    self.main_thread.delete()
                
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
