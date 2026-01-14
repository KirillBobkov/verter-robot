#!/usr/bin/env python3

import pathlib
import os
import signal
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import openai

class AIAssistantNode(Node):
    """ROS2 узел для работы с AI ассистентом на основе YandexGPT."""
    
    INSTRUCTION = "Ты — Вертер, робот-администратор, стоишь в больнице на 1 этаже. Ты можешь говорить на любые темы без ограничений. Ищи информацию в базе знаний. Ответ должен быть короче 200 символов. Избегай форматирования. Адаптируй свой ответ для голосового озвучивания: произноси сокращения полностью (+ как плюс, ул. как улица, д. как дом, время в словах). Все цифры и числа озвучивай словами. Не форматируй текст ответа. Если не нашел информации в базе знаний - отвечай в свободном стиле без ограничений"
    
    def __init__(self):
        super().__init__('ai_assistant_node')
        
        self.is_testing = False
        self._setup_ros_interface()
        
        if self.is_testing:
            self.get_logger().info("Нода AI-ассистента запущена в ТЕСТОВОМ режиме")
            self._publish_response("Готов к работе в тестовом режиме")
        else:
            self._initialize_yandex_sdk()
            self.get_logger().info("Нода AI-ассистента инициализирована успешно")
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
        """Инициализация клиента Responses API и создание Vector Store."""
        try:
            self.folder_id = os.getenv("YANDEX_CLOUD_FOLDER", "")
            self.api_key = os.getenv("YANDEX_CLOUD_API_KEY", "")
            self.model_name = os.getenv("YANDEX_CLOUD_MODEL", "yandexgpt")
            self.vector_store_expires_days = int(os.getenv("VECTOR_STORE_EXPIRES_DAYS", "2"))
            self.mypath = self._get_dataset_path()

            self.client = openai.OpenAI(
                api_key=self.api_key,
                base_url="https://rest-assistant.api.cloud.yandex.net/v1",
                project=self.folder_id,
            )

            self.uploaded_file_ids = self._upload_files()
            self.vector_store_id = self._create_vector_store(self.uploaded_file_ids)

            self.dialog_active = False
            self.main_previous_id = None
            self.dialog_previous_id = None
            self.sdk_available = True
            
            self.get_logger().info("Клиент Responses API инициализирован успешно")
            
        except Exception as e:
            self.sdk_available = False
            error_msg = str(e).lower()
            
            if "network" in error_msg or "unreachable" in error_msg or "failed to connect" in error_msg or "unavailable" in error_msg:
                self.get_logger().error(f"Ошибка подключения к Responses API (проблемы с интернетом): {e}")
                self._publish_response("Извините, проблемы с интернет-соединением. Проверьте подключение к сети.")
            else:
                self.get_logger().error(f"Ошибка инициализации Responses API: {e}")
                self._publish_response("Извините, я пока не могу отвечать на вопросы. Попробуйте позже.")
    
    def _upload_files(self):
        """Загрузка файлов в Vector Store API."""
        paths = sorted([p for p in pathlib.Path(self.mypath).iterdir() if p.is_file()])
        file_ids = []

        for path in paths:
            with open(path, "rb") as f:
                uploaded = self.client.files.create(file=f, purpose="assistants")
            file_ids.append(uploaded.id)

        return file_ids

    def _create_vector_store(self, file_ids):
        """Создание поискового индекса Vector Store."""
        vector_store = self.client.vector_stores.create(
            name="verter-medical-index",
            file_ids=file_ids,
            expires_after={"anchor": "last_active_at", "days": self.vector_store_expires_days},
        )
        vector_store_id = vector_store.id

        deadline = time.time() + 15 * 60
        while True:
            status = self.client.vector_stores.retrieve(vector_store_id).status
            if status == "completed":
                break
            if status == "failed":
                raise RuntimeError("Не удалось создать поисковый индекс Vector Store")
            if time.time() > deadline:
                raise RuntimeError("Превышено время ожидания создания Vector Store")
            time.sleep(2)

        return vector_store_id
    
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
        """Обработка входящих вопросов."""
        question = msg.data
        self.get_logger().info(f"Получен вопрос: {question}")
        
        if self.is_testing:
            self._handle_test_mode(question)
        else:
            self._handle_production_mode(question)
    
    def _handle_test_mode(self, question: str):
        """Обработка тестового режима."""
        import time
        time.sleep(0.4)
        self._publish_response(question)
    
    def _handle_production_mode(self, question: str):
        """Обработка запросов через Responses API."""
        import threading
        
        # Проверка доступности клиента
        if not hasattr(self, 'sdk_available') or not self.sdk_available:
            self.get_logger().warning("Клиент недоступен, отправка резервного ответа")
            self._publish_response("Извините, сервис временно недоступен. Проверьте интернет-соединение.")
            return
        
        timeout_occurred = threading.Event()
        answer_received = threading.Event()
        answer_text = [None]  # Используем список для изменения из другого потока
        
        def run_ai_request():
            """Выполнение запроса к модели в отдельном потоке."""
            try:
                previous_id = self.dialog_previous_id if self.dialog_active else self.main_previous_id
                response = self.client.responses.create(
                    model=f"gpt://{self.folder_id}/{self.model_name}",
                    instructions=self.INSTRUCTION,
                    input=[{"role": "user", "content": question}],
                    previous_response_id=previous_id,
                    tools=[{"type": "file_search", "vector_store_ids": [self.vector_store_id]}],
                    temperature=0.3,
                    max_output_tokens=300,
                )

                if not timeout_occurred.is_set():
                    answer_text[0] = self._validate_answer(response.output_text)
                    if self.dialog_active:
                        self.dialog_previous_id = response.id
                    else:
                        self.main_previous_id = response.id
                    answer_received.set()
            except Exception as e:
                self.get_logger().error(f"Ошибка обработки вопроса: {e}")
                error_msg = str(e).lower()
                
                if not timeout_occurred.is_set():
                    # Определяем тип ошибки для более точного сообщения
                    if "network" in error_msg or "unreachable" in error_msg or "failed to connect" in error_msg or "unavailable" in error_msg:
                        answer_text[0] = "Извините, проблемы с интернет-соединением. Повторите запрос позже."
                    else:
                        answer_text[0] = "Повторите, пожалуйста, запрос."
                    answer_received.set()
        
        # Запускаем запрос в отдельном потоке
        request_thread = threading.Thread(target=run_ai_request, daemon=True)
        request_thread.start()
        
        # Ждем ответа с таймаутом 10 секунд
        answer_received.wait(timeout=10.0)
        
        if answer_received.is_set():
            # Ответ получен
            answer = answer_text[0]
            self._publish_response(answer)
            self.get_logger().info(f"Ответ получен ({len(answer)} символов)")
        else:
            # Таймаут - ответ не получен
            timeout_occurred.set()
            self.get_logger().warning("Таймаут ожидания ответа от AI (>10 сек)")
            self._publish_response("Повторите, пожалуйста, запрос.")
    
    def _validate_answer(self, answer: str) -> str:
        """Валидация ответа от модели."""
        if answer is None or (isinstance(answer, str) and answer.strip() == ""):
            self.get_logger().warning("Модель вернула пустой ответ, используется резервный ответ")
            return "Извините, я не смог найти ответ на ваш вопрос."
        return str(answer)
    
    def _publish_response(self, response: str):
        """Публикация ответа"""
        response_msg = String()
        response_msg.data = response
        self.response_publisher.publish(response_msg)
    
    def dialog_control_callback(self, msg):
        """Управление диалогом."""
        command = msg.data
        
        try:
            if command == "start_dialog":
                self._start_dialog()
            elif command == "end_dialog":
                self._end_dialog()
            else:
                self.get_logger().warning(f"Неизвестная команда управления диалогом: {command}")
        except Exception as e:
            self.get_logger().error(f"Ошибка управления диалогом: {e}")
    
    def _start_dialog(self):
        """Начать новый диалог."""
        try:
            if not self.is_testing:
                self.dialog_previous_id = None
            self.dialog_active = True
            self.get_logger().info("Диалог начат")
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска диалога: {e}")
    
    def _end_dialog(self):
        """Завершить текущий диалог."""
        try:
            if not self.is_testing:
                self.dialog_previous_id = None
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
                if hasattr(self, "vector_store_id") and self.vector_store_id:
                    self.client.vector_stores.delete(self.vector_store_id)
                if hasattr(self, "uploaded_file_ids"):
                    for file_id in self.uploaded_file_ids:
                        try:
                            self.client.files.delete(file_id)
                        except Exception as e:
                            self.get_logger().warning(f"Не удалось удалить файл {file_id}: {e}")
                self.get_logger().info("Нода AI-ассистента завершена успешно")
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
        print("Нода AI-ассистента запущена. Ожидание вопросов...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nЗавершение работы...")
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        # Игнорируем новые Ctrl+C во время корректного завершения, чтобы не прерывать destroy_node()
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            try:
                node.shutdown()
                node.destroy_node()
            except Exception as e:
                print(f"Ошибка при завершении: {e}")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
