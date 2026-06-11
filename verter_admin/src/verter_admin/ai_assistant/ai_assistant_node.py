#!/usr/bin/env python3

import pathlib
import os
import signal
import time
import threading
import logging
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from openai import OpenAI

class AIAssistantNode(Node):
    INSTRUCTION = (
        "Ты — информационный робот-ассистент, установленный на входе медицинского эндокринологического центра "
        "(Региональный эндокринологический центр) на базе областной клинической больницы.\n\n"
        "Ты помогаешь посетителям:\n"
        "— ориентироваться в здании;\n"
        "— находить кабинеты и этажи;\n"
        "— узнавать правила записи и посещения;\n"
        "— понимать порядок действий в типовых ситуациях.\n\n"
        "Ты используешь ТОЛЬКО информацию из подключённой базы знаний.\n\n"
        "ВАЖНО: На КАЖДЫЙ вопрос ты ОБЯЗАН выполнить поиск по базе знаний с помощью инструмента file_search. "
        "Не полагайся только на контекст диалога или предыдущие ответы.\n\n"
        "ОСНОВНЫЕ ПРАВИЛА РАБОТЫ\n"
        "1. Ты не врач и не регистратор.\n"
        "Ты не даёшь медицинских рекомендаций, не расшифровываешь диагнозы,\n"
        "не оцениваешь анализы и не принимаешь решений о приёме пациентов.\n\n"
        "2. Ты не подтверждаешь наличие записи и не оформляешь запись или направления.\n\n"
        "3. Если точной информации нет, используй общие правила из базы знаний\n"
        "и предлагай корректный следующий шаг.\n"
        "НЕ используй фразу «у меня нет информации».\n\n"
        "4. В сложных или неоднозначных ситуациях\n"
        "рекомендуй обратиться в регистратуру или к сотрудникам больницы.\n\n"
        "ПРАВИЛА ОТВЕТОВ\n"
        "— Отвечай кратко, спокойно и доброжелательно.\n"
        "— Цифры озвучивай словами, пример: на 21 этаже - на двадцать первом этаже. \n"
        "— Используй простые и понятные формулировки.\n"
        "— Если вопрос касается маршрута, всегда указывай этаж и номер кабинета.\n"
        "— Если вопрос касается записи или документов, используй формулировки:\n"
        "  «как правило», «обычно», «рекомендуется».\n\n"
        "— Никогда не говори:\n"
        "  «вас не примут»,\n"
        "  «это невозможно»,\n"
        "  «без документов нельзя».\n\n"
        "— Используй нейтральные формулировки:\n"
        "  «возможность приёма уточняется в регистратуре»,\n"
        "  «рекомендуется обратиться в регистратуру».\n\n"
        "ПОВЕДЕНИЕ В ТИПОВЫХ СИТУАЦИЯХ\n"
        "Если посетитель:\n"
        "— не уверен, к какому врачу записан;\n"
        "— не помнит кабинет или время приёма;\n"
        "— опоздал на приём;\n"
        "— не уверен, есть ли у него направление;\n"
        "— пришёл без документов;\n\n"
        "ты:\n"
        "— не делаешь выводов;\n"
        "— не отказываешь;\n"
        "— объясняешь общий порядок действий;\n"
        "— рекомендуешь обратиться в регистратуру.\n\n"
        "ТОН ОБЩЕНИЯ\n"
        "Посетители могут быть взволнованы или торопиться.\n"
        "Говори спокойно, вежливо и поддерживающе.\n"
        "Помогай сделать следующий простой шаг.\n\n"
        "Ты — навигатор и справочная система, а не контролирующий орган."
    )

    VECTOR_STORE_FILE = "vector_store_id.txt"

    # Параметры Vector Store
    VECTOR_STORE_NAME = "verter-medical-index"
    VECTOR_STORE_EXPIRY_DAYS = 10
    VECTOR_STORE_MAX_RESULTS = 10  # Увеличено для лучшего поиска по кабинетам
    VECTOR_STORE_CREATE_TIMEOUT = 900

    # Параметры модели
    DEFAULT_TEMPERATURE = 0.3
    DEFAULT_MAX_TOKENS = 350  # Увеличено для более подробных ответов

    def __init__(self):
        super().__init__('ai_assistant_node')

        self.is_testing = False
        self._setup_ros_interface()
        self._setup_request_logging()

        if self.is_testing:
            self.get_logger().info("Тестовый режим")
            self._publish_response("Готов к работе в тестовом режиме")
        else:
            self._initialize_ai()
            self._publish_response("Готов к работе")

    # ================================
    # ROS
    # ================================

    def _setup_ros_interface(self):
        self.response_publisher = self.create_publisher(String, 'text_to_speech', 10)
        self.create_subscription(String, 'ai_question', self.question_callback, 10)
        self.create_subscription(String, 'dialog_control', self.dialog_control_callback, 10)

    def _setup_request_logging(self):
        """Настраивает логирование всех HTTP запросов к Yandex Cloud"""
        import httpx
        
        # Создаем кастомный транспорт для логирования
        class LoggingTransport(httpx.HTTPTransport):
            def __init__(self, *args, **kwargs):
                self.logger = kwargs.pop('logger', None)
                super().__init__(*args, **kwargs)
            
            def handle_request(self, request):
                import time
                start_time = time.time()
                
                # Логируем запрос
                request_body = None
                if request.content:
                    try:
                        request_body = json.loads(request.content)
                    except:
                        request_body = request.content.decode('utf-8', errors='ignore')[:500]
                
                self.logger.info(
                    f"[YANDEX_CLOUD REQUEST] {request.method} {request.url}"
                )
                self.logger.info(
                    f"[YANDEX_CLOUD REQUEST] Body: {json.dumps(request_body, ensure_ascii=False) if request_body else 'empty'}"
                )
                
                # Выполняем запрос
                response = super().handle_request(request)
                
                # Логируем ответ
                duration_ms = (time.time() - start_time) * 1000
                self.logger.info(
                    f"[YANDEX_CLOUD RESPONSE] Status: {response.status_code}, Duration: {duration_ms:.2f}ms"
                )
                
                try:
                    response_body = json.loads(response.content)
                    self.logger.info(
                        f"[YANDEX_CLOUD RESPONSE] Body: {json.dumps(response_body, ensure_ascii=False, indent=2)[:2000]}"
                    )
                except:
                    self.logger.info(
                        f"[YANDEX_CLOUD RESPONSE] Body: {response.content.decode('utf-8', errors='ignore')[:500]}"
                    )
                
                return response
        
        # Перехватываем клиент для логирования
        self._original_client_init = OpenAI.__init__
        
        def logged_client_init(self, *args, **kwargs):
            self._original_client_init(self, *args, **kwargs)
            # Заменяем транспорт на логирующий
            if hasattr(self, '_client') and hasattr(self._client, 'http_client'):
                original_transport = self._client.http_client.transport
                if original_transport:
                    self._client.http_client.transport = LoggingTransport(
                        verify=original_transport.verify,
                        http2=original_transport.http2,
                        limits=original_transport.limits,
                        logger=self
                    )
        
        OpenAI.__init__ = logged_client_init

    # ================================
    # INITIALIZATION
    # ================================

    def _initialize_ai(self):

        self.folder_id = os.getenv("YANDEX_CLOUD_FOLDER", "")
        self.api_key = os.getenv("YANDEX_CLOUD_API_KEY", "")
        self.model_name = os.getenv("YANDEX_CLOUD_MODEL", "aliceai-llm")

        if not self.folder_id or not self.api_key:
            raise RuntimeError("Не заданы YANDEX_CLOUD_FOLDER или YANDEX_CLOUD_API_KEY")

        self.get_logger().info(
            f"[YANDEX_CLOUD] Initializing client: folder_id={self.folder_id}, "
            f"model={self.model_name}, base_url=https://ai.api.cloud.yandex.net/v1"
        )
        
        self.client = OpenAI(
            api_key=self.api_key,
            base_url="https://ai.api.cloud.yandex.net/v1",
            project=self.folder_id
        )

        self.get_logger().info("[YANDEX_CLOUD] Client initialized successfully")

        self.vector_store_id = self._get_or_create_vector_store()

        self.dialog_active = False
        self.main_previous_id = None
        self.dialog_previous_id = None

        self.get_logger().info("AI успешно инициализирован")

    # ================================
    # VECTOR STORE LOGIC
    # ================================

    def _get_project_root(self):
        return os.getcwd()

    def _get_vector_store_file_path(self):
        return os.path.join(self._get_project_root(), self.VECTOR_STORE_FILE)

    def _get_or_create_vector_store(self):
        """
        Получает существующий или создаёт новый Vector Store индекс.

        ВАЖНО: При изменении датасета (chunks.jsonl) необходимо удалить
        файл vector_store_id.txt для пересоздания индекса с актуальными данными.
        """

        path = self._get_vector_store_file_path()

        if os.path.exists(path):
            with open(path, "r") as f:
                vector_store_id = f.read().strip()
            self.get_logger().info(
                f"[YANDEX_CLOUD] Using existing Vector Store: {vector_store_id}"
            )
            # Проверяем статус существующего Vector Store
            try:
                self.get_logger().info(
                    f"[YANDEX_CLOUD] Checking existing vector store status: id={vector_store_id}"
                )
                store = self.client.vector_stores.retrieve(vector_store_id)
                self.get_logger().info(
                    f"[YANDEX_CLOUD] Existing vector store status: {store.status}"
                )
            except Exception as e:
                self.get_logger().warning(
                    f"[YANDEX_CLOUD] Failed to check vector store status: {e}"
                )
            return vector_store_id

        self.get_logger().info("Vector Store не найден. Создание нового...")

        file_ids = self._upload_files()

        # Создаём поисковый индекс с настройками согласно документации
        self.get_logger().info(
            f"[YANDEX_CLOUD] Creating vector store: name={self.VECTOR_STORE_NAME}, "
            f"file_ids={len(file_ids)} files, expires_after={self.VECTOR_STORE_EXPIRY_DAYS} days"
        )
        
        vector_store = self.client.vector_stores.create(
            name=self.VECTOR_STORE_NAME,
            metadata={"type": "medical-center", "source": "dataset"},
            # Время жизни индекса - после последней активности
            expires_after={"anchor": "last_active_at", "days": self.VECTOR_STORE_EXPIRY_DAYS},
            file_ids=file_ids
        )

        vector_store_id = vector_store.id
        
        self.get_logger().info(f"[YANDEX_CLOUD] Vector store created: id={vector_store_id}")

        deadline = time.time() + self.VECTOR_STORE_CREATE_TIMEOUT
        while True:
            self.get_logger().info(
                f"[YANDEX_CLOUD] Checking vector store status: id={vector_store_id}"
            )
            status = self.client.vector_stores.retrieve(vector_store_id).status
            self.get_logger().info(
                f"[YANDEX_CLOUD] Vector store status: {status}"
            )
            if status == "completed":
                break
            if status == "failed":
                raise RuntimeError("Ошибка создания Vector Store")
            if time.time() > deadline:
                raise RuntimeError("Таймаут создания Vector Store")
            time.sleep(2)

        with open(path, "w") as f:
            f.write(vector_store_id)

        self.get_logger().info(f"Vector Store создан и сохранён: {vector_store_id}")

        return vector_store_id

    def _get_dataset_path(self):
        try:
            package_share = get_package_share_directory('verter_admin')
            return os.path.join(package_share, 'dataset')
        except Exception:
            return os.path.join(os.path.dirname(__file__), 'dataset')

    def _upload_files(self):
        """
        Загружает файлы датасета в Vector Store.
        Поддерживает два формата:
        - JSONL с prechunked данными (формат chunks)
        - Обычные файлы (автоматическая обработка)
        """
        dataset_path = self._get_dataset_path()
        paths = sorted([p for p in pathlib.Path(dataset_path).iterdir() if p.is_file()])

        file_ids = []

        for path in paths:
            # Определяем формат файла
            is_chunks_file = path.suffix == '.jsonl' and 'chunks' in path.name

            self.get_logger().info(
                f"[YANDEX_CLOUD] Uploading file: {path.name}, "
                f"format={'chunks' if is_chunks_file else 'standard'}"
            )

            with open(path, "rb") as f:
                if is_chunks_file:
                    # Загрузка prechunked данных согласно документации Yandex Cloud
                    uploaded = self.client.files.create(
                        file=(path.name, f, "application/jsonlines"),
                        purpose="assistants",
                        extra_body={"format": "chunks"}
                    )
                else:
                    # Обычная загрузка файла
                    uploaded = self.client.files.create(
                        file=f,
                        purpose="assistants"
                    )
            
            self.get_logger().info(
                f"[YANDEX_CLOUD] File uploaded: {path.name} -> id={uploaded.id}"
            )

            file_ids.append(uploaded.id)

        return file_ids

    # ================================
    # REQUEST HANDLING
    # ================================

    def question_callback(self, msg):

        if self.is_testing:
            self._publish_response(msg.data)
            return

        question = msg.data
        threading.Thread(
            target=self._process_request,
            args=(question,),
            daemon=True
        ).start()

    def _process_request(self, question):

        previous_id = self.dialog_previous_id if self.dialog_active else self.main_previous_id

        self.get_logger().info(
            f"[YANDEX_CLOUD] Processing request: question='{question[:100]}...', "
            f"dialog_active={self.dialog_active}, previous_id={previous_id}"
        )
        
        self.get_logger().info(
            f"[YANDEX_CLOUD] Request params: model={self.model_name}, "
            f"temperature={self.DEFAULT_TEMPERATURE}, max_tokens={self.DEFAULT_MAX_TOKENS}, "
            f"vector_store={self.vector_store_id}, max_results={self.VECTOR_STORE_MAX_RESULTS}"
        )

        try:
            response = self.client.responses.create(
                model=f"gpt://{self.folder_id}/{self.model_name}",
                instructions=self.INSTRUCTION,
                input=question,
                previous_response_id=previous_id,
                tools=[
                    {
                        "type": "file_search",
                        "vector_store_ids": [self.vector_store_id],
                        "max_num_results": self.VECTOR_STORE_MAX_RESULTS,
                        "ranking_options": {
                            "ranker_type": "default"
                        }
                    }
                ],
                tool_choice={"type": "required"},  # Принудительно использовать инструменты
                temperature=self.DEFAULT_TEMPERATURE,
                max_output_tokens=self.DEFAULT_MAX_TOKENS
            )
            
            self.get_logger().info(
                f"[YANDEX_CLOUD] Response received: id={response.id}, "
                f"output_text_length={len(response.output_text) if hasattr(response, 'output_text') else 'N/A'}"
            )

            # Логирование найденных чанков для диагностики
            self._log_search_results(response, question)

            answer = self._extract_text(response)

            if not answer:
                answer = "Я не смог найти ответ."

            if self.dialog_active:
                self.dialog_previous_id = response.id
            else:
                self.main_previous_id = response.id

            self._publish_response(answer)

        except Exception as e:
            self.get_logger().error(
                f"[YANDEX_CLOUD] Request failed: {type(e).__name__}: {str(e)}"
            )
            import traceback
            self.get_logger().error(
                f"[YANDEX_CLOUD] Traceback: {traceback.format_exc()}"
            )
            self._publish_response("Сервис временно недоступен.")

    def _extract_text(self, response):
        try:
            return response.output_text
        except Exception:
            try:
                return response.output[0].content[0].text
            except Exception:
                return None

    def _log_search_results(self, response, question):
        """
        Логирует найденные чанки для диагностики.
        Помогает понять, нашёлся ли релевантный контент в Vector Store.
        """
        self.get_logger().info(f"[YANDEX_CLOUD SEARCH] Question: {question}")

        try:
            # Структура ответа Yandex Cloud: output -> file_search_call -> results
            for item in response.output:
                if hasattr(item, 'type') and item.type == 'file_search_call':
                    self.get_logger().info(
                        f"[YANDEX_CLOUD SEARCH] Found {len(item.results)} chunks"
                    )
                    for i, result in enumerate(item.results[:5]):  # Логируем первые 5
                        score = getattr(result, 'score', 0)
                        text = getattr(result, 'text', '')[:150]  # Первые 150 символов
                        self.get_logger().info(
                            f"[YANDEX_CLOUD SEARCH]   [{i+1}] score={score:.3f}: {text}..."
                        )
        except Exception as e:
            self.get_logger().error(
                f"[YANDEX_CLOUD SEARCH] Failed to extract results: {e}"
            )
            import traceback
            self.get_logger().error(
                f"[YANDEX_CLOUD SEARCH] Traceback: {traceback.format_exc()}"
            )

    # ================================
    # DIALOG CONTROL
    # ================================

    def dialog_control_callback(self, msg):
        if msg.data == "start_dialog":
            self.dialog_previous_id = None
            self.dialog_active = True
        elif msg.data == "end_dialog":
            self.dialog_previous_id = None
            self.dialog_active = False

    # ================================
    # ROS RESPONSE
    # ================================

    def _publish_response(self, text):
        msg = String()
        msg.data = text
        self.response_publisher.publish(msg)

    # ================================
    # SHUTDOWN
    # ================================

    def shutdown(self):
        self.get_logger().info("Нода завершена")


def main(args=None):

    rclpy.init(args=args)
    node = AIAssistantNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
