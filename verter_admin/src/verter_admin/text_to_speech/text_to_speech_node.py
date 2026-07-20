#!/usr/bin/env python3

import os
import subprocess
import threading
import io
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

try:
    from piper import PiperVoice
    import wave
    from ru_normalizr import Normalizer, NormalizeOptions
except ImportError:
    PiperVoice = None
    wave = None
    Normalizer = None
    NormalizeOptions = None

class TextToSpeechNode(Node):
    
    def __init__(self):
        super().__init__('text_to_speech_node')
        
        if PiperVoice is None or wave is None:
            self.get_logger().error("piper-tts не установлен. Установите: pip install piper-tts")
            raise ImportError("piper-tts не найден")
        
        # Найти модель Piper
        try:
            package_share = get_package_share_directory('verter_admin')
            tts_models_dir = os.path.join(package_share, 'text_to_speech')
            self.model_path = os.path.join(tts_models_dir, "ru_RU-ruslan-medium.onnx")
        except Exception as e:
            self.get_logger().error(f"Модель не найдена: {e}")
            raise
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Файл модели не найден: {self.model_path}")
            raise FileNotFoundError("Модель Piper не найдена")
        
        self.declare_parameter('audio_device', 'pulse')
        self.audio_device = self.get_parameter('audio_device').get_parameter_value().string_value
        
        self._setup_environment()
        self._initialize_tts()
        
        # ROS2 подписки
        self.subscription = self.create_subscription(String, 'text_to_speech', self.text_callback, 10)
        # Publisher для управления распознаванием (и speech_to_text, и speech_recognition)
        self.tts_control_pub = self.create_publisher(Bool, 'tts_control', 10)
        
        # Управление конкуренцией
        self._busy_lock = threading.Lock()
        self._busy = False
        self._pending_text = None
        
        # Отслеживание текущего aplay процесса
        self.current_aplay_pid = None

        self.get_logger().info(f"Piper TTS готов. Модель: {self.model_path}")
    
    def _initialize_tts(self):
        # Загружаем модель через PiperVoice API
        try:
            self.voice = PiperVoice.load(self.model_path)
            self.get_logger().info(f"Piper модель загружена: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки модели: {e}")
            raise

        # Инициализация ru-normalizr для контекстного склонения числительных (режим TTS)
        if Normalizer is None:
            self.get_logger().warning("ru-normalizr не установлен. Установите: pip install ru-normalizr")
            self.normalizer = None
        else:
            try:
                self.normalizer = Normalizer(NormalizeOptions.tts())
                self.get_logger().info("ru-normalizr инициализирован")
            except Exception as e:
                self.get_logger().error(f"Ошибка инициализации нормализатора: {e}")
                self.normalizer = None
    
    def _setup_environment(self):
        self.env = os.environ.copy()
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")
        
        # Увеличиваем задержку для стабильности при автозапуске
        self.env["PULSE_LATENCY_MSEC"] = "100"
        
        # Убедимся, что XDG_RUNTIME_DIR установлен для PulseAudio
        if "XDG_RUNTIME_DIR" not in self.env:
            self.env["XDG_RUNTIME_DIR"] = f"/run/user/{os.getuid()}"
        
        self.get_logger().info(f"Audio device: {self.audio_device}")
        self.get_logger().info(f"XDG_RUNTIME_DIR: {self.env.get('XDG_RUNTIME_DIR')}")
        self.get_logger().info(f"PULSE_LATENCY_MSEC: {self.env.get('PULSE_LATENCY_MSEC')}")
    
    def text_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return
        
        start_new = False
        with self._busy_lock:
            if self._busy:
                self._pending_text = text
            else:
                self._busy = True
                start_new = True

        if start_new:
            threading.Thread(target=self._speak_loop, args=(text,), daemon=True).start()

    def _normalize_text(self, text):
        """Нормализация текста для TTS с контекстным склонением (годы, даты, числа, аббревиатуры)"""
        if not text or self.normalizer is None:
            return text

        try:
            # ru-normalizr (режим TTS) обрабатывает: годы, даты, время, валюты, проценты,
            # сокращения, римские цифры, аббревиатуры, латиницу с контекстным склонением
            normalized = self.normalizer.normalize(text)
            return normalized
        except Exception as e:
            self.get_logger().warning(f"ru-normalizr ошибка: {e}, возвращаем оригинал")
            return text

    def _speak_loop(self, first_text):
        current_text = first_text
        try:
            while current_text:
                try:
                    self._synthesize_and_play(current_text)
                except Exception as e:
                    self.get_logger().error(f"Ошибка синтеза: {e}")
                    self._activate_speech_recognition()

                with self._busy_lock:
                    current_text = self._pending_text
                    self._pending_text = None
        finally:
            with self._busy_lock:
                self._busy = False
    
    def _synthesize_and_play(self, text):
        try:
            # СНАЧАЛА ОТКЛЮЧАЕМ МИКРОФОН
            self._deactivate_speech_recognition()

            # Нормализация текста (годы, даты, числа, аббревиатуры)
            self.get_logger().info(f"Исходный текст: {text[:100]}...")
            normalized = self._normalize_text(text)
            if normalized != text:
                self.get_logger().info(f"Нормализация текста: \"{text[:50]}...\" → \"{normalized[:50]}...\"")
            else:
                self.get_logger().info("Текст не требует нормализации")
            text = normalized

            self.get_logger().info(f"Текст для синтеза: {text[:100]}...")
            sample_rate = str(self.voice.config.sample_rate)
            self.get_logger().info(f"🔊 TTS: Синтез текста: '{text[:100]}...'")

            # Запускаем aplay СРАЗУ, без ожидания данных
            # Увеличенные размеры буфера для предотвращения underrun при автозапуске
            cmd = ["aplay", "-D", self.audio_device, "-q", "-f", "S16_LE", "-r", sample_rate, "-c", "1", "--buffer-size=4096", "--period-size=512"]
            self.get_logger().info(f"Запуск aplay: {' '.join(cmd)}")
            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                bufsize=0,  # Отключаем буферизацию stdin
                env=self.env
            )

            chunk_count = 0
            total_bytes = 0

            # Синтезируем и сразу отправляем в aplay
            self.get_logger().info("🔊 TTS: Начало синтеза...")
            for chunk in self.voice.synthesize(text.strip()):
                if process.poll() is not None:
                    self.get_logger().warning(f"🔊 TTS: aplay завершился досрочно на чанке {chunk_count}")
                    break

                try:
                    process.stdin.write(chunk.audio_int16_bytes)
                    chunk_count += 1
                    total_bytes += len(chunk.audio_int16_bytes)
                except BrokenPipeError:
                    self.get_logger().error(f"🔊 TTS: BrokenPipeError на чанке {chunk_count}")
                    break
                except Exception as e:
                    self.get_logger().error(f"🔊 TTS: Ошибка записи на чанке {chunk_count}: {e}")
                    break

            self.get_logger().info(f"🔊 TTS: Синтез завершён, {chunk_count} чанков, {total_bytes} байт")

            process.stdin.close()

            # Ждем завершения
            self.get_logger().info("🔊 TTS: Ожидание завершения aplay...")
            return_code = process.wait()
            stderr_output = process.stderr.read().decode() if process.stderr else ""

            if return_code != 0:
                self.get_logger().error(f"🔊 TTS: aplay завершился с кодом {return_code}, stderr: {stderr_output}")
            elif stderr_output:
                # Логируем предупреждения даже при успешном завершении
                self.get_logger().warning(f"🔊 TTS: aplay stderr: {stderr_output}")
                self.get_logger().info("✓ Аудио воспроизведено успешно")
            else:
                self.get_logger().info("✓ Аудио воспроизведено успешно")

        except Exception as e:
            self.get_logger().error(f"🔊 TTS: Ошибка воспроизведения: {e}")
            import traceback
            self.get_logger().error(f"🔊 TTS: Traceback: {traceback.format_exc()}")
        finally:
            # ВКЛЮЧАЕМ МИКРОФОН ОБРАТНО С ЗАДЕРЖКОЙ (предотвращение эхо)
            import time
            time.sleep(0.1)  # Минимальная задержка для затухания звука
            self._activate_speech_recognition()
    
    
    def _deactivate_speech_recognition(self):
        """Отключить распознавание перед TTS"""
        msg = Bool()
        msg.data = False
        self.tts_control_pub.publish(msg)
        self.get_logger().info("🔇 Отключаю микрофон перед TTS")
    
    def _activate_speech_recognition(self):
        """Включить распознавание после TTS"""
        msg = Bool()
        msg.data = True
        self.tts_control_pub.publish(msg)
        self.get_logger().info("🎤 Включаю микрофон после TTS")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TextToSpeechNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
