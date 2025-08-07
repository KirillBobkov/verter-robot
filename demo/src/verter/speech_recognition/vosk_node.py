#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import vosk
import sounddevice as sd
import queue
import json
import os
import threading
import time
from ament_index_python.packages import get_package_share_directory
import pyttsx3
import signal
import sys
import psutil
import subprocess
import os
import numpy as np

class VoskNode(Node):
    def __init__(self):
        super().__init__('vosk_node')
        
        # Инициализация базовых атрибутов
        self.publisher_ = self.create_publisher(String, '/verter_commands', 10)
        self.sound_publisher_ = self.create_publisher(String, '/verter_say', 10)
        self.audio_queue_ = queue.Queue()
        self.shutdown_event_ = threading.Event()
        self._triggered = False
        self._shutting_down = False
        self.not_recognized_counter = 0
        
        # Флаг для управления распознаванием речи во время воспроизведения звука
        self.audio_in_progress = False
        
        # Подписка на топик состояния воспроизведения звука
        self.audio_status_subscription = self.create_subscription(
            String,
            '/audio_in_progress',
            self.audio_status_callback,
            10
        )
        
        # Настройка компонентов
        self._setup_command_map()
        self._setup_parameters()
        self._setup_vosk()
        self._start_audio_capture()
        
        # Запуск обработки
        self.process_timer = self.create_timer(0.1, self.process_audio)
        self.get_logger().info("VoskNode инициализирован успешно")
        
        # Флаг готовности (будет установлен когда микрофон подключится)
        self.microphone_ready = False
        
        # Установка обработчиков сигналов для корректного завершения
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _setup_command_map(self):
        """Инициализация словаря команд"""
        self.command_map_ = {
            "CHASSIS:STOP": ['стой', 'остановись', 'остановились', 'стоп', 'хватит', 'прекрати', 'замри', 'не двигайся', 'тормози'],
            "CHASSIS:MOVE_FORWARD:255": ['вперёд', 'прямо', 'езжай', 'поехали', 'двигайся вперёд', 'марш', 'вперед'],
            "CHASSIS:ROTATE_LEFT:255": ['поверни налево', 'повернись налево'],
            "CHASSIS:ROTATE_RIGHT:255": ['поверни направо', 'повернись направо'],
            "CHASSIS:MOVE_BACKWARD:255": ['назад', 'обратно', 'повернись назад', 'двигайся назад', 'отступить'],
            "CHASSIS:DANCE": ['танцуй', 'танцуйте', 'танцуйся', 'танцуйтесь', 'танцуй', 'танцуйте', 'танцуйся', 'танцуйтесь', 'танцуй', 'танцуйте', 'танцуйся', 'танцуйтесь'],

            "ARM_RIGHT:WAVE": ['помаши рукой','помаши правой рукой', 'памахай рукой', 'помахай рукой', 'помахай правой рукой', 'помахай рукой', 'помахай правой', 'помахай'],
            "ARM_RIGHT:RAISE": ['подними правую руку', 'правую руку вверх'],
            "ARM_LEFT:RAISE": ['подними левую руку', 'левую руку вверх'],
            "ARM_BOTH:RAISE": ['подними обе руки', 'обе руки вверх', 'оба рука верх', 'обе руки верх'],
            "ARM_BOTH:CALIBRATE": ['сделай калибровку', 'калибровка', 'откалибруйся', 'выполни калибровку', 'запусти калибровку'],

            "ARM_LEFT:BRUSH_OPEN": ["открой левую кисть", "разожми левую кисть", "раскрой левую кисть"],
            "ARM_LEFT:BRUSH_CLOSE": ["сожми левую кисть", "схвати левой рукой", "закрой левую кисть", "закрой левый захват"],
            "ARM_RIGHT:BRUSH_OPEN": ["открой правую кисть", "разожми правую кисть", "раскрой правую кисть"],
            "ARM_RIGHT:BRUSH_CLOSE": ["сожми правую кисть", "схвати правую рукой", "закрой правую кисть", "закрой правый захват"]
        }
        
        # Словарь звуковых команд
        self.sound_command_map_ = {
            'say_about_myself': ['привет расскажи о себе', 'расскажи о себе', 'ты кто', 'кто ты', 'что ты умеешь'],
            'hello': ['привет', 'как дела'],
            'about': [
                'расскажи о компании', 'откуда ты' 'как называется ваша компания', 'название компании', 'как называется компания'
            ],
            'stop_sound': ['молчи', 'молчать', 'замолчи', 'хватит'],
            'joke': ['расскажи шутку', 'расскажи анекдот', 'шутку', 'анекдот', 'пошути'],

            'thanks': ['спасибо', 'благодарю'],
            'goodbye': ['до свидания', 'пока'],
            'cool': ['ты классный', 'ты крутой', 'ты супер'],
            'understand': ['понимаешь меня', 'меня понимаешь', 'меня слышишь', 'слышишь меня' ],
            'yandex': ['как', ' тебе москва'],
            'cleaner': ['зачем', 'отпуск'],
            # 'civileva': ['анна', 'евгеньевна', 'шевелева', 'цивилёва', 'цивилева', 'председатель фонда защитников', 'поприветствуй анну евгеньевну', 'глава фонда защитников отечества'],
            # 'habrov': ['хабра', 'юрий', 'храбрых', 'хабров', 'анатольевич' 'юрий анатольевич хабров', 'директор фонда защитники отечества', 'поприветствуй юрия анатольевича', 'юрий анатольевич', 'подошел юрий анатольевич'],
            # 'zashitniki': ['защитником', 'отечества', 'фонд', 'защитники', 'защитники', 'защитников', 'защитников отечества', 'защитники отечества'],
            # 'minpromtorg': ['антона', 'андреевича', 'антон', 'андреевич', 'алиханов', 'подошел антон андреевич'],
            # "minpromtorg_2": ['минпромторг', "промышленность и торговлю", "мин пром торг", "промышленность", 'торговля', 'торговлю'],
            # "mintruda": ["соцзащиты","социальной защиты", "министерство труда", "минтруда", "министерство соцзащиты"],
            # 'minzdorovya': ["здравоохранение", "минздрав", "здравоохранения", "минздрава"],
            # 'motorika': ["моторика", "это моторика", "моторику", 'это из моторики'],
            # "motorika_head": ['андрей', "давидюк", 'павлович', "давидюк андрей павлович", "андрей павлович", "директор моторики", "это директор моторики"],
            # "motorika_perfomance": ["выступлении моторики","выступление моторики", 'выступлениями моторики'],
            # "metiz": ['митис', "метиз", "метиc"],
            'help': ['помоги', 'мне плохо', 'я чувствую себя плохо'],
            'help_2': ['упал'],
            'help_3': ['температура', 'озноб'],
            # 'fadviz': ['таганрог', 'таганрог', 'фадвиз', 'фадвис'],
            # 'metallist': ['металист','металлист', 'металлиста'],
            # 'chast_chaloveka': ['валя', 'леша', 'лёша', 'валя и лёша', 'часть человека'],
            # 'cito': ['ци то','цито', 'цветов', 'ростех'],
            'bashni': ['башни', 'башня', 'башню'],
            'moskva_reka': ['реке', 'река', 'реки'],
            'results': ['подведи', 'итоги', 'выставки'],
            # 'sergey': ['обращение', 'cергею', 'иннокентьевичу'],
            # 'hi_trip': ['наших', 'людей', 'наши', 'люди'],
            # 'hi_karina': ['поздравление', 'карине', 'андреевне', 'каринэ', 'андревне'],
            'kovalev': ['владимир', 'владимиру', 'вячеславовичу', 'вячеславовичу', 'ковалёву', 'ковалёв', 'ковалев'],
            'gavrin': ['сергею', 'александровичу', 'александрович', 'гаврину', 'гаврин', 'сергей'],
            'next_procedure': ['когда', 'следующая', 'процедура'],
            'exit': ['осталось', 'выписки'],
            'abramov': ['курс', 'лечения'],
            'eng': ['по английски', 'английский', 'по-английски'],
            'follow': ['следишь', 'пациентами'],
            'kirillova': ['наталье кирилловой', 'наталья кириллова', 'наталья', 'наталье', 'наталья кирилловой'],
            'benefit': ['польза']
        }
        self.trigger_words_ = ['робот', 'робат', 'бертом', 'верстах', 'вертэр', 'вертер', 'ветер', 'ертер', 'верте', 'вектор', 'ветера', 'мерсер', 'вертера', 'вердер', 'лестер']

    def _find_gm303_device(self):
        """Автоматический поиск GM303 устройства с fallback на PulseAudio"""
        try:
            devices = sd.query_devices()
            
            # Сначала ищем GM303 напрямую
            gm303_device = None
            for i, device in enumerate(devices):
                if 'GM303' in str(device['name']) and device['max_input_channels'] > 0:
                    gm303_device = i
                    self.get_logger().info(f"GM303 найден: device {i} - {device['name']}")
                    break
            
            # Пробуем подключиться к GM303 напрямую
            if gm303_device is not None:
                try:
                    sd.check_input_settings(device=gm303_device, samplerate=44100, channels=1)
                    self.get_logger().info(f"✓ GM303 device {gm303_device} доступен напрямую")
                    return gm303_device
                except Exception as e:
                    self.get_logger().warn(f"GM303 device {gm303_device} занят: {e}, переключаемся на PulseAudio")
            
            # Fallback на PulseAudio
            for i, device in enumerate(devices):
                if device['name'] == 'pulse' and device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ Используем PulseAudio: device {i}")
                    return i
            
            # Fallback на default
            for i, device in enumerate(devices):
                if device['name'] == 'default' and device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ Используем default: device {i}")
                    return i
            
            # Последний fallback
            self.get_logger().warn("Устройства не найдены, используется device 5 (pulse)")
            return 5
            
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска устройства: {e}, используется device 5")
            return 5

    def _setup_parameters(self):
        """Настройка локальных параметров"""
        self.model_name_ = 'vosk-model-small-ru-0.22'
        self.mic_samplerate_ = 44100  # Частота микрофона
        self.vosk_samplerate_ = 44100  # Используем нативную частоту для лучшего качества
        self.channels_ = 1
        self.device_ = self._find_gm303_device()  # Автоматический поиск GM303
        vosk_log_level = -1
        vosk.SetLogLevel(vosk_log_level)

    def _setup_vosk(self):
        """Инициализация модели и распознавателя Vosk"""
        model_path = self._resolve_model_path()
        if not model_path:
            raise RuntimeError("Failed to resolve model path")
            
        try:
            self.model_ = vosk.Model(model_path)
            self.recognizer_ = vosk.KaldiRecognizer(self.model_, self.vosk_samplerate_)
            # Включаем частичные результаты для лучшей точности
            self.recognizer_.SetWords(True)
            self.get_logger().info(f"Модель Vosk загружена из: {model_path}")
        except Exception as e:
            raise RuntimeError(f"Ошибка загрузки модели Vosk: {e}")

    def _resolve_model_path(self):
        """Определение полного пути к модели Vosk"""
        if os.path.isabs(self.model_name_):
            model_path = self.model_name_
        else:
            try:
                package_share = get_package_share_directory('verter')
                model_path = os.path.join(package_share, self.model_name_)
            except Exception as e:
                self.get_logger().error(f"Пакет 'verter' не найден: {e}")
                return None
        
        if not os.path.isdir(model_path):
            self.get_logger().error(f"Директория модели не найдена: {model_path}")
            return None
            
        return model_path

    def _start_audio_capture(self):
        """Запуск потока захвата аудио с ожиданием USB микрофона"""
        self.audio_thread_ = threading.Thread(target=self._audio_capture_loop_with_retry)
        self.audio_thread_.daemon = True
        self.audio_thread_.start()

    def _audio_capture_loop_with_retry(self):
        """Цикл захвата аудио с повторными попытками подключения к микрофону"""
        max_retries = 50  # Увеличили до 50 попыток
        retry_delay = 5   # Увеличили до 5 секунд
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"Попытка подключения к аудио #{attempt + 1}/{max_retries}")
                
                # Ищем лучшее доступное устройство
                self.device_ = self._find_gm303_device()
                
                self._audio_capture_loop()
                return  # Успешное подключение
                
            except Exception as e:
                self.get_logger().warn(f"Попытка #{attempt + 1} неудачна: {e}")
                
                # Очищаем аудиосистему перед следующей попыткой
                try:
                    sd.stop()
                    time.sleep(1)
                except:
                    pass
                    
                if attempt < max_retries - 1:
                    self.get_logger().info(f"Ожидание {retry_delay} секунд перед следующей попыткой...")
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error("Не удалось подключиться к аудиоустройству после всех попыток")
                    return

    def _audio_capture_loop(self):
        """Основной цикл захвата аудио"""
        try:
            device_info = sd.query_devices(self.device_, 'input')
            self.get_logger().info(f"Используется аудиоустройство: {device_info['name']}")
            
            # Остановим все активные потоки перед началом
            try:
                sd.stop()
                time.sleep(1)
            except:
                pass
            
            # Проверка поддержки sample rate 44100
            try:
                sd.check_input_settings(device=self.device_, samplerate=self.mic_samplerate_, channels=self.channels_)
                self.get_logger().info(f"✓ GM303 поддерживает {self.mic_samplerate_} Hz, {self.channels_} канал")
            except Exception as e:
                self.get_logger().error(f"Устройство не поддерживает {self.mic_samplerate_} Hz: {e}")
                return

            # Простое подключение без дополнительных параметров
            with sd.RawInputStream(
                samplerate=self.mic_samplerate_,
                blocksize=4410,
                device=self.device_,
                dtype='int16',
                channels=self.channels_,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Захват аудио GM303 запущен успешно")
                # Отправляем сигнал готовности когда микрофон действительно готов
                self._send_microphone_ready_signal()
                self.shutdown_event_.wait()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка захвата аудио: {e}")
            raise  # Перебрасываем ошибку для retry

    def _audio_callback(self, indata, frames, time, status):
        """Callback для аудиоданных"""
        if status:
            self.get_logger().warn(f"Статус аудио: {status}")
        # Не добавляем данные в очередь если идет воспроизведение звука или происходит завершение
        if not self.shutdown_event_.is_set() and not self.audio_in_progress:
            self.audio_queue_.put(bytes(indata))

    def _resample_audio(self, data):
        """Усиление аудиосигнала без ресэмплинга для лучшего качества"""
        try:
            # Конвертация bytes в numpy array
            audio_data = np.frombuffer(data, dtype=np.int16)
            
            # Увеличение громкости микрофона на 30%
            audio_data = audio_data.astype(np.float32)
            audio_data *= 1.3
            # Ограничение чтобы избежать переполнения
            audio_data = np.clip(audio_data, -32768, 32767)
            audio_data = audio_data.astype(np.int16)
            
            return audio_data.tobytes()
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки аудио: {e}")
            return data
        
    def process_audio(self):
        """Обработка аудиоданных из очереди"""
        if self.shutdown_event_.is_set() or self.audio_in_progress:
            return
            
        try:
            data = self.audio_queue_.get(block=True, timeout=0.1)
        except queue.Empty:
            return

        try:
            # Обработка аудио перед отправкой в Vosk
            processed_data = self._resample_audio(data)
            
            if self.recognizer_.AcceptWaveform(processed_data):
                result = json.loads(self.recognizer_.Result())
                text = result.get('text', '').strip()
                
                if text:
                    self.get_logger().info(f"Распознано: {text}")
                    self._process_speech_command(text)
                    
        except Exception as e:
            self.get_logger().error(f"Ошибка распознавания: {e}")

    def _process_speech_command(self, text):
        """Обработка распознанной речи для команд"""
        text_lower = text.lower()
        
        # Проверка триггерных слов
        if self._has_trigger_word(text_lower):
            self._triggered = True
            self.get_logger().info("Активирован триггером")

        # Обработка команды, если активирован триггер
        if self._triggered:
            # Проверка звуковых команд
            sound_command = self._find_sound_command(text_lower)
            if sound_command:
                self._send_sound_command(sound_command)
                self._triggered = False
                return
                
            # Проверка обычных команд
            command = self._find_command(text_lower)
            if command:
                self._send_command(command)
                self._triggered = False
            else:
                # Команда не найдена - отправляем сообщение о нераспознавании
                self._triggered = False
                self.get_logger().info("Команда не найдена, триггер сброшен")
                # Чередование команд notrecognized и repeatplease
                self.not_recognized_counter += 1
                if self.not_recognized_counter % 2 == 1:
                    command_name = 'notrecognized'
                else:
                    command_name = 'repeatplease'
                
                repeat_msg = String()
                repeat_msg.data = command_name
                self.sound_publisher_.publish(repeat_msg)
                self.get_logger().info(f'Отправлена команда {command_name}')

    def _has_trigger_word(self, text):
        """Проверка наличия триггерных слов в тексте"""
        return any(trigger in text for trigger in self.trigger_words_)

    def _find_command(self, text):
        """Поиск подходящей команды в тексте"""
        for command, keywords in self.command_map_.items():
            if any(keyword in text for keyword in keywords):
                return command
        return None

    def _find_sound_command(self, text):
        """Поиск подходящей звуковой команды в тексте"""
        for command, keywords in self.sound_command_map_.items():
            if any(keyword in text for keyword in keywords):
                return command
        return None

    def _send_command(self, command):
        """Отправка сообщения с командой"""
        # Сначала воспроизводим звук doing.mp3
        sound_msg = String()
        sound_msg.data = 'doing.mp3'
        self.sound_publisher_.publish(sound_msg)
        self.get_logger().info('Воспроизводится звук doing.mp3')
        
        # Отправляем соответствующую команду для глаз перед основной командой
        eye_commands = {
            "ARM_RIGHT:WAVE": "EYES:RIGHT",
            "ARM_RIGHT:RAISE": "EYES:RIGHT",
            "ARM_LEFT:RAISE": "EYES:LEFT",
            "ARM_BOTH:RAISE": "EYES:UP",
            "ARM_BOTH:CALIBRATE": "EYES:CENTER",
            "ARM_LEFT:BRUSH_OPEN": "EYES:LEFT",
            "ARM_LEFT:BRUSH_CLOSE": "EYES:LEFT",
            "ARM_RIGHT:BRUSH_OPEN": "EYES:RIGHT",
            "ARM_RIGHT:BRUSH_CLOSE": "EYES:RIGHT",
            "CHASSIS:DANCE": "ARM_BOTH:DANCE",
        }
        
        if command in eye_commands:
            eye_msg = String()
            eye_msg.data = eye_commands[command]
            self.publisher_.publish(eye_msg)
            self.get_logger().info(f'Отправлена команда для глаз: {eye_commands[command]}')
            # Небольшая задержка перед отправкой основной команды
            time.sleep(0.2)
        
        # Отправляем основную команду роботу
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Команда роботу отправлена в /verter_commands: {command}')

    def _send_sound_command(self, command):
        """Отправка звуковой команды"""
        # Команды приветствия, которые должны сопровождаться маханием рукой
        greeting_commands = ['kovalev', 'gavrin', 'hello'];
        hello_commands = ['hello'];
        
        # Если это команда приветствия, отправляем команду махания рукой
        if command in greeting_commands:
            wave_msg = String()
            wave_msg.data = 'ARM_RIGHT:WAVE'
            self.publisher_.publish(wave_msg)
            self.get_logger().info(f'Отправлена команда махания рукой: ARM_RIGHT:WAVE')
            # Небольшая задержка перед отправкой звуковой команды
            time.sleep(0.2)
            greeting_commands = ['hi_trip', 'hi_kurgan', 'hi_karina', 'sergey']
        
        # # Если это команда приветствия, отправляем команду махания рукой
        # if command in greeting_commands:
        #     wave_msg = String()
        #     wave_msg.data = 'EYES:LEFT'
        #     self.publisher_.publish(wave_msg)
        #     self.get_logger().info(f'Отправлена команда махания рукой: EYES:LEFT')
        #     # Небольшая задержка перед отправкой звуковой команды
        #     time.sleep(0.2)
        
        if command in hello_commands:
            wave_msg = String()
            wave_msg.data = 'EYES:CENTER'
            self.publisher_.publish(wave_msg)
            self.get_logger().info(f'Отправлена команда махания рукой: EYES:LEFT')
            # Небольшая задержка перед отправкой звуковой команды
            time.sleep(0.2)


        # Отправляем звуковую команду
        msg = String()
        msg.data = command
        self.sound_publisher_.publish(msg)
        self.get_logger().info(f'Отправлена звуковая команда: {command}')

    def _send_microphone_ready_signal(self):
        """Отправка сигнала готовности микрофона"""
        if not self.microphone_ready:
            self.microphone_ready = True
            msg = String()
            msg.data = 'ready'
            self.sound_publisher_.publish(msg)
            self.get_logger().info('Микрофон готов - отправлен сигнал ready')

    def _signal_handler(self, signum, frame):
        """Обработчик сигналов завершения"""
        self.shutdown()
        
    def shutdown(self):
        """Завершение работы узла"""
        if getattr(self, '_shutting_down', False):
            return
            
        self._shutting_down = True
        self.shutdown_event_.set()
        
        # Остановка компонентов
        self._stop_timers()
        self._stop_audio()
        self._clear_queue()
        self._cleanup_vosk()
        self._kill_processes()
    
    def _stop_timers(self):
        """Остановка таймеров"""
        for timer_name in ['process_timer', 'ready_timer']:
            timer = getattr(self, timer_name, None)
            if timer:
                timer.cancel()
    
    def _stop_audio(self):
        """Остановка аудио"""
        try:
            sd.stop()
        except:
            pass
        
        if hasattr(self, 'audio_thread_'):
            self.audio_thread_.join(timeout=1.0)
    
    def _clear_queue(self):
        """Очистка очереди"""
        while not self.audio_queue_.empty():
            try:
                self.audio_queue_.get_nowait()
            except:
                break
    
    def _cleanup_vosk(self):
        """Освобождение Vosk ресурсов"""
        for attr in ['recognizer_', 'model_']:
            if hasattr(self, attr):
                delattr(self, attr)
    
    def _kill_processes(self):
        """Убийство всех связанных процессов"""
        # Убиваем дочерние процессы
        try:
            current_process = psutil.Process()
            for child in current_process.children(recursive=True):
                child.kill()
        except:
            pass
        
        # Убиваем по PID группе
        try:
            os.killpg(os.getpgrp(), signal.SIGTERM)
        except:
            pass
    
    def __enter__(self):
        """Поддержка контекстного менеджера"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Автоматическая очистка при выходе из контекста"""
        self.shutdown()
        return False

    def audio_status_callback(self, msg):
        """Обработчик состояния воспроизведения звука"""
        new_status = msg.data == 'True'
        if new_status != self.audio_in_progress:
            self.audio_in_progress = new_status
            if self.audio_in_progress:
                # Очищаем очередь при начале воспроизведения
                while not self.audio_queue_.empty():
                    try:
                        self.audio_queue_.get_nowait()
                    except queue.Empty:
                        break
                # Сбрасываем состояние триггера
                self._triggered = False
                self.get_logger().info("Воспроизведение звука началось - распознавание речи отключено, очередь очищена, триггер сброшен")
            else:
                # Очищаем состояние распознавателя при окончании воспроизведения
                try:
                    self.recognizer_.Reset()
                except:
                    pass
                self.get_logger().info("Воспроизведение звука завершено - распознавание речи включено, распознаватель сброшен")

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = VoskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()