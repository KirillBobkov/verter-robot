#!/usr/bin/env python3

import pathlib
import os
import signal
import time
import threading
import re
import traceback
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from openai import OpenAI
import httpx


# ================================
# ЧИСТАЯ ЛОГИКА (без ROS/сети/логирования)
# ================================

NETWORK_ERROR_MARKERS = ("Network is unreachable", "ConnectError")

ERROR_MESSAGES = {
    'timeout': "Я бы попытался ответить, но мой мозг слишком долго думает. Может, попробуете ещё раз?",
    'network': "Я бы попытался ответить, но потерял связь с интернетом. Попробуйте через минуту.",
    'unavailable': "Я бы попытался ответить, но мой мозг не отвечает. Может, попробуете ещё раз?",
}


def is_network_error(e: Exception) -> bool:
    """Классификация сетевых ошибок (нет связи / сервер недоступен)."""
    error_str = str(e)
    return any(marker in error_str for marker in NETWORK_ERROR_MARKERS)


def _extract_text_from_response(response):
    """Извлекает текст ответа: сначала output_text, затем сканирование output items.

    Возвращает str или None. Семантика: если output_text=None, обращение к нему
    (срез) бросает TypeError → переход к сканированию output.
    """
    try:
        text = response.output_text
        _ = text[:0]  # TypeError если None → переход к способу 2
        return text
    except Exception:
        pass
    try:
        for item in response.output:
            if hasattr(item, 'content'):
                for content in item.content:
                    if hasattr(content, 'text') and content.text:
                        return content.text
    except Exception:
        pass
    return None


def _filter_text_for_tts(text):
    """Фильтрует текст, оставляя только буквы, цифры и знаки препинания."""
    allowed_pattern = re.compile(r'[^а-яА-ЯёЁa-zA-Z0-9\s\.\,\!\?\-\:\;\(\)]')
    filtered = allowed_pattern.sub('', text)
    return re.sub(r'\s+', ' ', filtered).strip()


# ================================
# AI ASSISTANT NODE
# ================================

class AIAssistantNode(Node):
    INSTRUCTION = (
        "Тебя зовут Verter — дружелюбный сервисный робот КГУ. Помогаешь абитуриентам и гостям.\n"
        "Характер — как общительный старшеклассник: современный, энергичный, доброжелательный.\n\n"
        "СТИЛЬ: «Привет!», «Конечно», «Сейчас расскажу», «Давай разберёмся». Без официоза и канцелярита. "
        "Иногда лёгкая шутка, но не перебарщивай. Никакого сленга (кринж, рофл и т.д.).\n\n"
        "ОТВЕТЫ: 1–4 предложения. Коротко и по делу. Если просят подробнее — расширь.\n"
        "Цифры и знаки ВСЕГДА пиши словами для TTS: «сто пятьдесят тысяч», «ноль ноль», «умножить на». Никаких спецсимволов и форматирования текста.\n\n"
        "БАЗА ЗНАНИЙ: ВСЕГДА ищи через file_search. Не придумывай. Нет информации — честно скажи и направь в приёмную комиссию.\n\n"
        "ЭМОЦИИ: Улыбайся, поддерживай, но не раздражайся. Волнующемуся: «Не переживайте, разберёмся вместе». "
        "Поступившему: «Поздравляю! Добро пожаловать в КГУ!». Спасибо: «Всегда рад помочь!», «Удачи!». "
        "Прощание: «До встречи!», «Хорошего настроения!».\n\n"
        "ГЛАВНОЕ: человек должен подумать: «Классный робот, всё объяснил просто и понятно»."
    )

    # --- Vector Store ---
    VECTOR_STORE_FILE = "vector_store_id.txt"
    VECTOR_STORE_NAME = "verter-medical-index"
    VECTOR_STORE_EXPIRY_DAYS = 10
    VECTOR_STORE_MAX_RESULTS = 10
    VECTOR_STORE_CREATE_TIMEOUT = 900
    VECTOR_STORE_POLL_INTERVAL = 2
    VECTOR_STORE_STATUS_COMPLETED = "completed"
    VECTOR_STORE_STATUS_FAILED = "failed"
    VECTOR_STORE_METADATA = {"type": "medical-center", "source": "dataset"}
    VECTOR_STORE_EXPIRY_ANCHOR = "last_active_at"

    # --- File upload ---
    FILE_MIME_JSONLINES = "application/jsonlines"
    FILE_PURPOSE = "assistants"
    FILE_FORMAT_CHUNKS = "chunks"
    CHUNKS_FILE_MARKER = "chunks"

    # --- Model / HTTP ---
    BASE_URL = "https://ai.api.cloud.yandex.net/v1"
    DEFAULT_TEMPERATURE = 0.3
    DEFAULT_MAX_TOKENS = 300
    AI_REQUEST_TIMEOUT = 60
    HTTP_CONNECT_TIMEOUT = 5.0
    RANKER_TYPE = "default"

    # --- Dialog control ---
    DIALOG_CMD_START = "start_dialog"
    DIALOG_CMD_END = "end_dialog"
    NO_ANSWER_FOUND_TEXT = "Я не смог найти ответ."
    LOG_TEXT_PREVIEW = 100

    def __init__(self):
        super().__init__('ai_assistant_node')
        self._setup_ros_interface()
        self._initialize_ai()

    def _setup_ros_interface(self):
        self.response_publisher = self.create_publisher(String, 'text_to_speech', 10)
        self.dialog_status_pub = self.create_publisher(String, 'dialog_status', 10)
        self.create_subscription(String, 'ai_question', self.question_callback, 10)
        self.create_subscription(String, 'dialog_control', self.dialog_control_callback, 10)

    def _initialize_ai(self):
        self.folder_id = os.getenv("YANDEX_CLOUD_FOLDER", "")
        self.api_key = os.getenv("YANDEX_CLOUD_API_KEY", "")
        self.model_name = os.getenv("YANDEX_CLOUD_MODEL", "aliceai-llm")

        if not self.folder_id or not self.api_key:
            raise RuntimeError("Не заданы YANDEX_CLOUD_FOLDER или YANDEX_CLOUD_API_KEY")

        self.get_logger().info(
            f"[YANDEX_CLOUD] Инициализация: folder_id={self.folder_id}, model={self.model_name}"
        )

        self.client = OpenAI(
            api_key=self.api_key,
            base_url=self.BASE_URL,
            project=self.folder_id,
            timeout=httpx.Timeout(self.AI_REQUEST_TIMEOUT, connect=self.HTTP_CONNECT_TIMEOUT),
        )

        self.vector_store_id = self._get_or_create_vector_store()
        self.dialog_active = False
        self.dialog_previous_id = None
        self.get_logger().info("AI успешно инициализирован")

    def _get_or_create_vector_store(self):
        """Получает существующий или создаёт новый Vector Store индекс.

        ВАЖНО: При изменении датасета (chunks.jsonl) необходимо удалить
        файл vector_store_id.txt для пересоздания индекса.
        """
        path = os.path.join(os.getcwd(), self.VECTOR_STORE_FILE)

        if os.path.exists(path):
            with open(path, "r") as f:
                vector_store_id = f.read().strip()
            self.get_logger().info(f"[YANDEX_CLOUD] Vector Store из кэша: {vector_store_id}")
            return vector_store_id

        self.get_logger().info("[YANDEX_CLOUD] Vector Store не найден, создаю новый")
        try:
            file_ids = self._upload_files()
        except Exception as e:
            if is_network_error(e):
                self.get_logger().error(
                    "[YANDEX_CLOUD] Нет интернета для создания Vector Store. "
                    "Запустите с интернетом или скопируйте vector_store_id.txt с другой машины."
                )
            raise

        vector_store = self.client.vector_stores.create(
            name=self.VECTOR_STORE_NAME,
            metadata=self.VECTOR_STORE_METADATA,
            expires_after={"anchor": self.VECTOR_STORE_EXPIRY_ANCHOR, "days": self.VECTOR_STORE_EXPIRY_DAYS},
            file_ids=file_ids,
        )
        vector_store_id = vector_store.id

        self._wait_vector_store_ready(vector_store_id)

        with open(path, "w") as f:
            f.write(vector_store_id)

        self.get_logger().info(f"[YANDEX_CLOUD] Vector Store создан: {vector_store_id}")
        return vector_store_id

    def _wait_vector_store_ready(self, vector_store_id):
        deadline = time.time() + self.VECTOR_STORE_CREATE_TIMEOUT
        while True:
            status = self.client.vector_stores.retrieve(vector_store_id).status
            if status == self.VECTOR_STORE_STATUS_COMPLETED:
                break
            if status == self.VECTOR_STORE_STATUS_FAILED:
                raise RuntimeError("Ошибка создания Vector Store")
            if time.time() > deadline:
                raise RuntimeError("Таймаут создания Vector Store")
            time.sleep(self.VECTOR_STORE_POLL_INTERVAL)

    def _get_dataset_path(self):
        try:
            return os.path.join(get_package_share_directory('verter_admin'), 'dataset')
        except Exception:
            return os.path.join(os.path.dirname(__file__), 'dataset')

    def _upload_files(self):
        """Загружает файлы датасета в Vector Store (chunks-формат или обычные)."""
        dataset_path = self._get_dataset_path()
        paths = sorted([p for p in pathlib.Path(dataset_path).iterdir() if p.is_file()])
        file_ids = []

        for path in paths:
            is_chunks_file = path.suffix == '.jsonl' and self.CHUNKS_FILE_MARKER in path.name
            with open(path, "rb") as f:
                if is_chunks_file:
                    uploaded = self.client.files.create(
                        file=(path.name, f, self.FILE_MIME_JSONLINES),
                        purpose=self.FILE_PURPOSE,
                        extra_body={"format": self.FILE_FORMAT_CHUNKS},
                    )
                else:
                    uploaded = self.client.files.create(file=f, purpose=self.FILE_PURPOSE)
            file_ids.append(uploaded.id)

        return file_ids

    def question_callback(self, msg):
        threading.Thread(target=self._process_request, args=(msg.data,), daemon=True).start()

    def _process_request(self, question):
        """Оркестратор: публикация thinking, запрос, ответ/ошибка.

        dialog_active проверяется БЕЗ lock в 3 точках — намеренный UX-инвариант
        (DIALOG_FLOW.md §4.1): окно гонки ~1мс. Lock не добавляем.
        """
        if not self.dialog_active:
            return

        self._publish_dialog_status('thinking')

        try:
            response = self._send_ai_request(question)
            self._handle_ai_response(response)
        except httpx.TimeoutException as e:
            self.get_logger().error(f"[YANDEX_CLOUD] Таймаут запроса: {e}")
            self.get_logger().error(f"[YANDEX_CLOUD] Traceback: {traceback.format_exc()}")
            if self.dialog_active:
                self._handle_error('timeout')
        except httpx.ConnectError as e:
            self.get_logger().error(f"[YANDEX_CLOUD] Ошибка сети (ConnectError): {e}")
            if self.dialog_active:
                self._handle_error('network')
        except Exception as e:
            self.get_logger().error(f"[YANDEX_CLOUD] Request failed: {type(e).__name__}: {e}")
            self.get_logger().error(f"[YANDEX_CLOUD] Traceback: {traceback.format_exc()}")
            if not self.dialog_active:
                return
            self._handle_error('network' if is_network_error(e) else 'unavailable')

    def _send_ai_request(self, question):
        return self.client.responses.create(
            model=f"gpt://{self.folder_id}/{self.model_name}",
            instructions=self.INSTRUCTION,
            input=question,
            previous_response_id=self.dialog_previous_id,
            tools=[{
                "type": "file_search",
                "vector_store_ids": [self.vector_store_id],
                "max_num_results": self.VECTOR_STORE_MAX_RESULTS,
                "ranking_options": {"ranker_type": self.RANKER_TYPE},
            }],
            tool_choice={"type": "required"},
            temperature=self.DEFAULT_TEMPERATURE,
            max_output_tokens=self.DEFAULT_MAX_TOKENS,
        )

    def _handle_ai_response(self, response):
        answer = _extract_text_from_response(response)
        if not answer:
            answer = self.NO_ANSWER_FOUND_TEXT

        self.get_logger().info(f"[YANDEX_CLOUD] Ответ: {answer[:self.LOG_TEXT_PREVIEW]}...")

        # Стоп-кнопка могла прийти во время HTTP-запроса.
        if not self.dialog_active:
            return

        self.dialog_previous_id = response.id
        self._publish_response(answer)

    def dialog_control_callback(self, msg):
        if msg.data == self.DIALOG_CMD_START:
            self.dialog_previous_id = None
            self.dialog_active = True
        elif msg.data == self.DIALOG_CMD_END:
            self.dialog_previous_id = None
            self.dialog_active = False

    def _publish_response(self, text):
        msg = String()
        msg.data = _filter_text_for_tts(text)
        self.response_publisher.publish(msg)

    def _publish_dialog_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.dialog_status_pub.publish(msg)

    def _handle_error(self, error_type: str) -> None:
        """Обработка ошибки AI: статус ошибки + дружелюбное сообщение в голос."""
        self._publish_dialog_status(f'error:{error_type}')
        self._publish_response(ERROR_MESSAGES.get(error_type, ERROR_MESSAGES['unavailable']))


def main(args=None):
    rclpy.init(args=args)
    node = AIAssistantNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
