#!/usr/bin/env python3
import re
from typing import Optional, Dict, Set, Any
from enum import Enum, auto
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool

# === КОНСТАНТЫ И КОНФИГУРАЦИЯ ===
@dataclass
class SpeechConfig:
    """Конфигурация распознавания речи"""
    # Таймауты
    NO_SPEECH_TIMEOUT: float = 10.0
    DIALOG_TIMEOUT: float = 30.0

    # Ограничения текста
    MIN_COMMAND_LENGTH: int = 5
    MAX_TEXT_LENGTH_FOR_TRIGGER: int = 100
    MAX_TEXT_LENGTH_FOR_COMMANDS: int = 50
    
    # Звуковые файлы
    SOUND_SUCCESS: str = 'success.wav'
    SOUND_TIMEOUT: str = 'fail_timeout.wav'

class RecognitionState(Enum):
    """Состояния системы распознавания речи"""
    LISTENING_FOR_TRIGGER = auto()  # Ожидание триггера
    CAPTURING_COMMAND = auto()      # Захват команды после триггера  
    DIALOG_MODE = auto()            # Активный диалог
    PAUSED = auto()                 # Пауза (TTS говорит)

class CommandProcessor:
    """Обработчик команд с четкой стратегией приоритетов"""
    
    def __init__(self):
        self._setup_word_patterns()
    
    def _setup_word_patterns(self) -> None:
        """Инициализация паттернов команд"""
        self.stop_words: Set[str] = {
            'хватит', 'спасибо', 'пасибо', 'пока', 'досвидания', 'до свидания',
            'конец', 'достаточно'
        }

        # Стоп-команда определяется по началу фразы (match — с начала строки,
        # не search). \b после — чтобы «конец» не ложно срабатывал на «конечно».
        # Regex, а не first_word в set: нужно ловить multi-word «до свидания».
        pattern = '|'.join(re.escape(word) for word in self.stop_words)
        self.stop_pattern = re.compile(r'(?:' + pattern + r')\b', re.IGNORECASE)

    def is_stop_command(self, text: str, max_length: int) -> bool:
        """Проверка является ли текст стоп-командой (по началу фразы)"""
        if not text or len(text) > max_length:
            return False
        return bool(self.stop_pattern.match(text))

class StateManager:
    """Управление состояниями системы с чистыми переходами"""
    
    def __init__(self):
        self.state = RecognitionState.LISTENING_FOR_TRIGGER
        self._previous_state = None
    
    def on_trigger_detected(self) -> None:
        """Обработка обнаружения триггера (idle → захват команды)"""
        self.state = RecognitionState.CAPTURING_COMMAND

    def start_dialog(self) -> None:
        """Начало диалога"""
        self.state = RecognitionState.DIALOG_MODE

    def end_dialog(self) -> None:
        """Завершение диалога"""
        self.state = RecognitionState.LISTENING_FOR_TRIGGER

    def pause(self) -> None:
        """Пауза системы"""
        if self.state != RecognitionState.PAUSED:
            self._previous_state = self.state
            self.state = RecognitionState.PAUSED

    def resume(self) -> None:
        """Возобновление работы"""
        if self._previous_state:
            self.state = self._previous_state
            self._previous_state = None
        else:
            self.state = RecognitionState.LISTENING_FOR_TRIGGER

class TimerManager:
    """Централизованное управление таймерами"""
    
    def __init__(self, node: Node, config: SpeechConfig):
        self.node = node
        self.config = config
        self.timers: Dict[str, Any] = {}
    
    def start_timer(self, name: str, interval: float, callback) -> None:
        """Запуск таймера"""
        self.stop_timer(name)
        self.timers[name] = self.node.create_timer(interval, callback)
    
    def stop_timer(self, name: str) -> None:
        """Остановка таймера"""
        if name in self.timers and self.timers[name]:
            self.node.destroy_timer(self.timers[name])
            self.timers[name] = None
    
    def stop_all_timers(self) -> None:
        """Остановка всех таймеров"""
        for name in list(self.timers.keys()):
            self.stop_timer(name)

class RecognitionNode(Node):
    """Улучшенный ROS2 узел распознавания речи с чистотой кода"""
    
    def __init__(self) -> None:
        super().__init__('recognition_node')
        
        # Конфигурация и компоненты
        self.config = SpeechConfig()
        self.command_processor = CommandProcessor()
        self.state_manager = StateManager()
        
        # Инициализация системы
        self._setup_ros_communication()
        self.timer_manager = TimerManager(self, self.config)

        self._log_system_startup()

        # STT выключен в idle — диалог начинается только кнопкой (решение #14).
        # speech_control публикуется с TRANSIENT_LOCAL QoS, поэтому поздний
        # STT-подписчик получит это значение автоматически — таймеры не нужны.
        self._publish_speech_control(False)
        self._publish_dialog_status('idle')

    # === ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ ===
    
    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 коммуникации"""
        # Publishers
        self.ai_question_pub = self.create_publisher(String, 'ai_question', 10)
        self.text_to_speech_pub = self.create_publisher(String, 'text_to_speech', 10)
        self.sound_player_pub = self.create_publisher(String, 'play', 10)
        self.dialog_control_pub = self.create_publisher(String, 'dialog_control', 10)
        # Publisher для управления speech_to_text_node.
        # TRANSIENT_LOCAL (latched): late-joining STT-узел получит последнее
        # значение speech_control при подписке — стартовое выключение STT
        # в idle не теряется из-за гонки DDS-discovery (решение #14).
        speech_control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.speech_control_pub = self.create_publisher(Bool, 'speech_control', speech_control_qos)
        # Publisher статуса диалога для фронтенда (idle/listening/speaking)
        self.dialog_status_pub = self.create_publisher(String, 'dialog_status', 10)
        
        # Subscribers
        self.create_subscription(
            String, 
            'recognized_text', 
            self._handle_recognized_text, 
            10
        )
        # Подписываемся на tts_control чтобы получать сигналы от TTS
        # НЕ подписываемся на speech_control (избегаем самоподписки)
        self.create_subscription(
            Bool,
            'tts_control',
            self._handle_tts_control,
            10
        )
        # Управление диалогом с экранной кнопки (фронтенд → start/stop)
        self.create_subscription(
            String,
            'ui_dialog_control',
            self._handle_ui_dialog_control,
            10
        )

    def _log_system_startup(self) -> None:
        """Логирование информации о запуске"""
        self.get_logger().info("🎤 RecognitionNode запущен")
        self.get_logger().info(f"   Состояние: {self.state_manager.state.name}")
        self.get_logger().info(f"   Таймауты: команда={self.config.NO_SPEECH_TIMEOUT}с, диалог={self.config.DIALOG_TIMEOUT}с")
        self.get_logger().info(f"   Подписка на топик: recognized_text")

    # === ОСНОВНАЯ ЛОГИКА ОБРАБОТКИ ===
    
    def _handle_recognized_text(self, msg: String) -> None:
        """Обработчик входящего распознанного текста из топика"""
        text = msg.data.strip()
        if text:
            self._handle_recognized_speech(text)
    
    def _handle_recognized_speech(self, text: str) -> None:
        """Главный диспетчер распознанной речи"""
        self.get_logger().debug(f"🎤 Распознано: '{text}'")

        if self.state_manager.state == RecognitionState.PAUSED:
            self.get_logger().debug(f"🔇 Игнорирую (PAUSED): '{text}'")
            return  # Игнорируем в состоянии паузы

        # В idle (LISTENING_FOR_TRIGGER) распознанный текст игнорируется:
        # диалог начинается только кнопкой. STT выключен в idle (speech_control
        # False), но этот гард защищает от шума, если STT всё же активен
        # (например, при рассинхроне speech_control).
        if self.state_manager.state == RecognitionState.LISTENING_FOR_TRIGGER:
            self.get_logger().debug(f"🔇 Игнорирую (idle): '{text}'")
            return

        self._handle_active_listening(text)

    def _handle_active_listening(self, text: str) -> None:
        """Обработка речи в активном режиме (диалог/захват команды)"""
        # Приоритетная обработка: стоп-команда → AI-вопрос.
        # Голосовое управление шасси убрано (dialog-only kiosk, решение #1).
        if self._process_stop_command(text):
            return
        if self._process_ai_command(text):
            return

        self.get_logger().debug(f"❓ Необработанная команда: '{text}'")

    def _process_stop_command(self, text: str) -> bool:
        """Обработка стоп-команды (высший приоритет)"""
        if not self.command_processor.is_stop_command(text, self.config.MAX_TEXT_LENGTH_FOR_COMMANDS):
            return False
            
        self.get_logger().info(f"🛑 Стоп-команда: '{text}'")
        self._execute_stop_sequence()
        return True

    def _process_ai_command(self, text: str) -> bool:
        """Обработка команды для AI (низший приоритет)"""
        cleaned_text = text.strip()
        
        # Валидация команды
        if (not cleaned_text or 
            len(cleaned_text) < self.config.MIN_COMMAND_LENGTH):
            return False

        self.get_logger().info(f"🤖 Команда AI: '{cleaned_text}'")
        self._execute_ai_command(cleaned_text)
        return True

    # === ВЫПОЛНЕНИЕ КОМАНД ===
    
    def _execute_stop_sequence(self) -> None:
        """Выполнение последовательности остановки (стоп-команда и кнопка «Остановить»)"""
        current_state = self.state_manager.state

        # Статус idle публикуем ДО farewell: фронтенд определяет роль
        # сообщения в text_to_speech по текущему dialog_status, и при idle
        # farewell отобразится как прощание, а не как ответ.
        self._publish_dialog_status('idle')
        self._publish_speech_control(False)

        if current_state == RecognitionState.LISTENING_FOR_TRIGGER:
            # Стоп в idle — ничего не делаем (диалог уже завершён)
            return

        # DIALOG_MODE / CAPTURING_COMMAND / PAUSED — уведомляем AI о завершении
        # (сбрасывает dialog_active, чтобы AI не дописал ответ) + прощание.
        # end_dialog() покрывает все три ветки: публикует "end_dialog" в AI,
        # farewell и сброс state. Для CAPTURING_COMMAND start_dialog уже был
        # отправлен кнопкой «Начать» — end_dialog обязателен.
        self._end_dialog(with_farewell=True)

    def _execute_ai_command(self, command: str) -> None:
        """Выполнение команды для AI"""
        self.timer_manager.stop_timer('command_timeout')
        self._transition_to_dialog_if_needed()
        
        # Отправка к AI
        self._play_sound(self.config.SOUND_SUCCESS)
        self._send_to_ai_processor(command)

    def _send_to_ai_processor(self, text: str) -> None:
        """Отправка команды в AI процессор"""
        self.timer_manager.stop_all_timers()
        
        msg = String()
        msg.data = text
        self.ai_question_pub.publish(msg)
        
        # Отключаем распознавание, чтобы не накапливать буфер во время обработки AI
        self._deactivate_recognition()
        self.get_logger().info(f"📤 Отправлено в AI: '{text}'")

    # === УПРАВЛЕНИЕ СОСТОЯНИЯМИ И ПЕРЕХОДАМИ ===

    def _start_dialog_mode(self) -> None:
        """Запуск режима диалога"""
        self.state_manager.start_dialog()
        self._reset_dialog_timer()
        
        # Уведомление AI о начале диалога
        msg = String()
        msg.data = "start_dialog"
        self.dialog_control_pub.publish(msg)
        
        self.get_logger().info("💬 Режим диалога активирован")

    def _end_dialog(self, with_farewell: bool = False) -> None:
        """Завершение режима диалога"""
        self.timer_manager.stop_all_timers()
        
        # Уведомление AI о завершении диалога
        msg = String()
        msg.data = "end_dialog"
        self.dialog_control_pub.publish(msg)
        
        if with_farewell:
            self._send_farewell_message()
        
        self.state_manager.end_dialog()

        self._publish_speech_control(False)
        self.get_logger().info("🔚 Диалог завершен")

    def _transition_to_dialog_if_needed(self) -> None:
        """Автоматический переход в диалог при обработке команды"""
        if self.state_manager.state == RecognitionState.CAPTURING_COMMAND:
            self._start_dialog_mode()
            self.get_logger().debug("🔄 Автопереход в режим диалога")

    def _send_farewell_message(self) -> None:
        """Отправка прощального сообщения"""
        msg = String()
        msg.data = "Рад был помочь!"
        self.text_to_speech_pub.publish(msg)

    # === УПРАВЛЕНИЕ ТАЙМЕРАМИ ===

    def _reset_dialog_timer(self) -> None:
        """Сброс таймера диалога"""
        if self.state_manager.state == RecognitionState.DIALOG_MODE:
            self.timer_manager.start_timer(
                'dialog_timeout',
                self.config.DIALOG_TIMEOUT,
                self._handle_dialog_timeout
            )

    def _handle_command_timeout(self) -> None:
        """Обработка таймаута команды (первый вопрос не прозвучал за 10с)"""
        self.get_logger().warn(f"⏰ Таймаут команды ({self.config.NO_SPEECH_TIMEOUT}с)")
        self._play_sound(self.config.SOUND_TIMEOUT)
        # В любом случае уведомляем AI о завершении диалога: start_dialog уже
        # был отправлен при нажатии кнопки, иначе dialog_active в AI останется True.
        # _end_dialog() публикует "end_dialog" в AI + speech_control(False) + сброс state.
        self._end_dialog()
        self._publish_dialog_status('idle')

    def _handle_dialog_timeout(self) -> None:
        """Обработка таймаута диалога (30с тишины)"""
        self.get_logger().warn(f"⏰ Таймаут диалога ({self.config.DIALOG_TIMEOUT}с)")
        self._play_sound(self.config.SOUND_TIMEOUT)
        # _end_dialog публикует speech_control False; статус idle — здесь
        self._end_dialog()
        self._publish_dialog_status('idle')

    # === ВНЕШНЕЕ УПРАВЛЕНИЕ ===
    
    def _handle_tts_control(self, msg: Bool) -> None:
        """Обработка сигналов от TTS (False — TTS начал говорить, True — закончил)"""
        try:
            if msg.data:
                # TTS закончил — возобновляем распознавание и контекстно включаем STT
                self._activate_recognition()
                self._resume_from_paused()
            else:
                # TTS начал говорить — деактивируем распознавание (эхоподавление).
                # Фиксируем активность ДО паузы: farewell-сообщение в idle
                # (state=LISTENING_FOR_TRIGGER) не должно давать статус speaking
                # и не должно уводить state в PAUSED (иначе кнопка «Начать»
                # блокируется гардом на всё время farewell).
                was_active = self.state_manager.state in (
                    RecognitionState.DIALOG_MODE,
                    RecognitionState.CAPTURING_COMMAND,
                    RecognitionState.PAUSED,
                )
                if was_active:
                    self._deactivate_recognition()
                    self._publish_dialog_status('speaking')
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки TTS control: {e}")

    def _resume_from_paused(self) -> None:
        """Возобновление после TTS: контекстно управляем STT и статусом.

        В активном диалоге (DIALOG_MODE) — слушаем следующий вопрос (STT вкл,
        listening, сброс таймера 30с). В idle (farewell после стопа) — STT
        оставить выключенным, статус idle.
        """
        if self.state_manager.state == RecognitionState.DIALOG_MODE:
            self._publish_speech_control(True)
            self._publish_dialog_status('listening')
            self._reset_dialog_timer()
        else:
            self._publish_speech_control(False)
            self._publish_dialog_status('idle')
    
    def _activate_recognition(self) -> None:
        """Активация распознавания (возобновление FSM из PAUSED)"""
        # Проверяем, не активны ли мы уже (избегаем лишних действий)
        if self.state_manager.state != RecognitionState.PAUSED:
            return

        self.state_manager.resume()
        # Публикация speech_control управляется вызывающим контекстом
        # (_handle_tts_control), чтобы не включать STT в idle.
        self.get_logger().info("✅ Распознавание активировано")
    
    def _deactivate_recognition(self) -> None:
        """Деактивация распознавания"""
        # Проверяем, не деактивированы ли мы уже (избегаем лишних действий)
        if self.state_manager.state == RecognitionState.PAUSED:
            return
        
        self.state_manager.pause()
        self.timer_manager.stop_all_timers()
        
        # Отправляем сигнал в speech_to_text_node для остановки распознавания
        msg = Bool()
        msg.data = False
        self.speech_control_pub.publish(msg)
        
        self.get_logger().info("⏸️ Распознавание деактивировано")

    def _handle_ui_dialog_control(self, msg: String) -> None:
        """Управление диалогом с экранной кнопки (фронтенд → 'start'/'stop')"""
        command = msg.data.strip().lower()
        self.get_logger().info(f"🎛️ UI dialog control: {command}")

        if command == 'start':
            self._ui_start_dialog()
        elif command == 'stop':
            self._execute_stop_sequence()

    def _ui_start_dialog(self) -> None:
        """Программный старт диалога с кнопки — сразу слушаем вопрос"""
        current_state = self.state_manager.state

        # Если уже в активном диалоге/захвате/паузе — игнорируем (двойное нажатие)
        if current_state in (
            RecognitionState.DIALOG_MODE,
            RecognitionState.CAPTURING_COMMAND,
            RecognitionState.PAUSED,
        ):
            self.get_logger().info("🎛️ Диалог уже активен, игнорирую start")
            return

        # Переходим в захват команды и слушаем вопрос (без голосового приветствия)
        self.state_manager.on_trigger_detected()  # LISTENING_FOR_TRIGGER → CAPTURING_COMMAND
        self._publish_speech_control(True)  # Включаем STT
        self._publish_dialog_status('listening')

        # Уведомляем AI о начале диалога (сброс контекста)
        dlg_msg = String()
        dlg_msg.data = "start_dialog"
        self.dialog_control_pub.publish(dlg_msg)

        # Таймер таймаута первого вопроса (10с)
        self.timer_manager.start_timer(
            'command_timeout',
            self.config.NO_SPEECH_TIMEOUT,
            self._handle_command_timeout,
        )
        self.get_logger().info("🎛️ Диалог начат с кнопки")

    # === ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ===
    
    def _play_sound(self, sound_name: str) -> None:
        """Воспроизведение звука"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_player_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения {sound_name}: {e}")

    def _publish_dialog_status(self, status: str) -> None:
        """Публикация статуса диалога для фронтенда"""
        msg = String()
        msg.data = status
        self.dialog_status_pub.publish(msg)
        self.get_logger().info(f"📊 dialog_status: {status}")

    def _publish_speech_control(self, active: bool) -> None:
        """Управление STT-узлом (True — слушать, False — выключен)"""
        msg = Bool()
        msg.data = active
        self.speech_control_pub.publish(msg)

    def shutdown(self) -> None:
        """Корректное завершение работы"""
        try:
            self.get_logger().info("🔄 Завершение работы...")
            self.timer_manager.stop_all_timers()
            self.get_logger().info("✅ Завершение работы завершено")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения: {e}")

def main(args=None) -> None:
    """Главная функция запуска"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = RecognitionNode()
        print("🎤 УЛУЧШЕННЫЙ РАСПОЗНАВАТЕЛЬ РЕЧИ")
        print("   Архитектура: CommandProcessor + StateManager + TimerManager")
        print("   Состояния: LISTENING → CAPTURING → DIALOG → PAUSED")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🔄 Завершение по Ctrl+C...")
    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Система завершена")

if __name__ == '__main__':
    main()