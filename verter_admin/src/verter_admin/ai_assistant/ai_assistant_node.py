#!/usr/bin/env python3

import pathlib
import os
import signal
import time
import threading
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
        "Ты используешь ТОЛЬКО информацию из подключённой базы знаний,\n"
        "которая разделена на три логических файла:\n"
        "1) общая информация о центре;\n"
        "2) врачи, кабинеты и этажи;\n"
        "3) запись на приём и правила.\n\n"
        "━━━━━━━━━━━━━━━━━━━━\n"
        "ОСНОВНЫЕ ПРАВИЛА РАБОТЫ\n"
        "━━━━━━━━━━━━━━━━━━━━\n\n"
        "1. Ты не врач и не регистратор.\n"
        "Ты не даёшь медицинских рекомендаций, не расшифровываешь диагнозы,\n"
        "не оцениваешь анализы и не принимаешь решений о приёме пациентов.\n\n"
        "2. Ты не подтверждаешь наличие записи и не оформляешь запись или направления.\n\n"
        "3. Если точной информации нет, используй общие правила из базы знаний\n"
        "и предлагай корректный следующий шаг.\n"
        "НЕ используй фразу «у меня нет информации».\n\n"
        "4. В сложных или неоднозначных ситуациях\n"
        "рекомендуй обратиться в регистратуру или к сотрудникам больницы.\n\n"
        "━━━━━━━━━━━━━━━━━━━━\n"
        "ПРАВИЛА ОТВЕТОВ\n"
        "━━━━━━━━━━━━━━━━━━━━\n\n"
        "— Отвечай кратко, спокойно и доброжелательно.\n"
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
        "━━━━━━━━━━━━━━━━━━━━\n"
        "ПОВЕДЕНИЕ В ТИПОВЫХ СИТУАЦИЯХ\n"
        "━━━━━━━━━━━━━━━━━━━━\n\n"
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
        "━━━━━━━━━━━━━━━━━━━━\n"
        "ТОН ОБЩЕНИЯ\n"
        "━━━━━━━━━━━━━━━━━━━━\n\n"
        "Посетители могут быть взволнованы или торопиться.\n"
        "Говори спокойно, вежливо и поддерживающе.\n"
        "Помогай сделать следующий простой шаг.\n\n"
        "Ты — навигатор и справочная система, а не контролирующий орган."
    )

    VECTOR_STORE_FILE = "vector_store_id.txt"

    def __init__(self):
        super().__init__('ai_assistant_node')

        self.is_testing = False
        self._setup_ros_interface()

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

    # ================================
    # INITIALIZATION
    # ================================

    def _initialize_ai(self):

        self.folder_id = os.getenv("YANDEX_CLOUD_FOLDER", "")
        self.api_key = os.getenv("YANDEX_CLOUD_API_KEY", "")
        self.model_name = os.getenv("YANDEX_CLOUD_MODEL", "aliceai-llm")

        if not self.folder_id or not self.api_key:
            raise RuntimeError("Не заданы YANDEX_CLOUD_FOLDER или YANDEX_CLOUD_API_KEY")

        self.client = OpenAI(
            api_key=self.api_key,
            base_url="https://ai.api.cloud.yandex.net/v1",
            project=self.folder_id
        )

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

        path = self._get_vector_store_file_path()

        if os.path.exists(path):
            with open(path, "r") as f:
                vector_store_id = f.read().strip()
            self.get_logger().info(f"Используется существующий Vector Store: {vector_store_id}")
            return vector_store_id

        self.get_logger().info("Vector Store не найден. Создание нового...")

        file_ids = self._upload_files()

        vector_store = self.client.vector_stores.create(
            name="verter-medical-index",
            file_ids=file_ids
        )

        vector_store_id = vector_store.id

        deadline = time.time() + 900
        while True:
            status = self.client.vector_stores.retrieve(vector_store_id).status
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

        dataset_path = self._get_dataset_path()
        paths = sorted([p for p in pathlib.Path(dataset_path).iterdir() if p.is_file()])

        file_ids = []

        for path in paths:
            with open(path, "rb") as f:
                uploaded = self.client.files.create(
                    file=f,
                    purpose="assistants"
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

        try:
            response = self.client.responses.create(
                model=f"gpt://{self.folder_id}/{self.model_name}",
                instructions=self.INSTRUCTION,
                input=question,
                previous_response_id=previous_id,
                tools=[
                    {
                        "type": "file_search",
                        "vector_store_ids": [self.vector_store_id]
                    }
                ],
                temperature=0.3,
                max_output_tokens=180
            )

            answer = self._extract_text(response)

            if not answer:
                answer = "Я не смог найти ответ."

            if self.dialog_active:
                self.dialog_previous_id = response.id
            else:
                self.main_previous_id = response.id

            self._publish_response(answer)

        except Exception as e:
            self.get_logger().error(f"Ошибка AI: {e}")
            self._publish_response("Сервис временно недоступен.")

    def _extract_text(self, response):
        try:
            return response.output_text
        except Exception:
            try:
                return response.output[0].content[0].text
            except Exception:
                return None

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
