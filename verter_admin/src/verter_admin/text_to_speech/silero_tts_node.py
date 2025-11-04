#!/usr/bin/env python3

import os
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

try:
    import torch
    import numpy as np
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    torch = None
    np = None


class SileroTTSNode(Node):
    
    def __init__(self):
        super().__init__('silero_tts_node')
        
        if not TORCH_AVAILABLE:
            self.get_logger().error("torch не установлен. Установите: pip install torch torchaudio")
            raise ImportError("torch не найден")
        
        # Путь к локальной модели (в той же директории что и нода)
        try:
            package_share = get_package_share_directory('verter_admin')
            tts_models_dir = os.path.join(package_share, 'text_to_speech')
            self.model_path = os.path.join(tts_models_dir, "v5_ru.pt")
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска модели: {e}")
            raise
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Файл модели не найден: {self.model_path}")
            raise FileNotFoundError("Модель Silero не найдена")
        
        # Настройки
        self.sample_rate = 24000
        self.speaker = 'eugene'
        self.put_accent = True
        self.put_yo = True
        self.put_stress_homo = True
        self.put_yo_homo = True
        
        self.audio_device = "pulse"
        self._setup_environment()
        self._initialize_tts()
        
        # ROS2 подписки
        self.subscription = self.create_subscription(String, 'text_to_speech', self.text_callback, 10)
        # Publisher для управления распознаванием
        self.tts_control_pub = self.create_publisher(Bool, 'tts_control', 10)
        
        # Управление конкуренцией
        self._busy_lock = threading.Lock()
        self._busy = False
        self._pending_text = None
        self._current_process = None  # Для возможности прерывания
        
        self.get_logger().info(f"Silero TTS готов. Sample rate: {self.sample_rate}, speaker: {self.speaker}")
    
    def _initialize_tts(self):
        """Загрузка локальной модели Silero"""
        try:
            self.get_logger().info(f"Загрузка локальной модели: {self.model_path}")
            start_load = time.time()
            
            # Загрузка локальной модели из .pt файла (как в примере с локальной моделью)
            importer = torch.package.PackageImporter(self.model_path)
            self.model = importer.load_pickle("tts_models", "model")
            
            load_time = time.time() - start_load
            self.get_logger().info(f"✓ Модель Silero загружена за {load_time:.2f} сек")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки модели: {e}")
            raise
    
    def _setup_environment(self):
        """Настройка окружения для аудио"""
        self.env = os.environ.copy()
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")
    
    def text_callback(self, msg):
        """Обработка входящих сообщений с текстом"""
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
    
    def _speak_loop(self, first_text):
        """Цикл обработки текстов с очередью"""
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
        """Синтез речи и воспроизведение через aplay"""
        process = None
        try:
            # СНАЧАЛА ОТКЛЮЧАЕМ МИКРОФОН
            self._deactivate_speech_recognition()
            
            self.get_logger().info(f"Синтез текста: {text[:50]}..." if len(text) > 50 else f"Синтез текста: {text}")
            start_synth = time.time()
            
            # Синтез речи (блокирующий, но необходим для Silero)
            audio = self.model.apply_tts(
                text=text.strip(),
                speaker=self.speaker,
                sample_rate=self.sample_rate,
                put_accent=self.put_accent,
                put_yo=self.put_yo,
                put_stress_homo=self.put_stress_homo,
                put_yo_homo=self.put_yo_homo
            )
            
            synth_time = time.time() - start_synth
            self.get_logger().info(f"✓ Синтез выполнен за {synth_time:.2f} сек")
            
            # Проверяем, не нужно ли прервать (новое сообщение в очереди)
            with self._busy_lock:
                if self._pending_text:
                    self.get_logger().info("⏭ Пропускаю текущее сообщение, есть новое в очереди")
                    return
            
            # Конвертация в int16 (оптимизированная)
            audio_np = audio.numpy()
            audio_int16 = (audio_np * 32767).astype(np.int16)
            audio_bytes = audio_int16.tobytes()
            
            # Запускаем aplay
            process = subprocess.Popen(
                [
                    "aplay",
                    "-D", self.audio_device,
                    "-q",
                    "-f", "S16_LE",
                    "-r", str(self.sample_rate),
                    "-c", "1",  # моно
                    "--buffer-size=8192"  # Увеличенный буфер для плавности
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                env=self.env
            )
            
            # Сохраняем ссылку для возможности прерывания
            with self._busy_lock:
                self._current_process = process
            
            # Передаем данные оптимальными чанками
            chunk_size = 8192  # Большие чанки для эффективности
            for i in range(0, len(audio_bytes), chunk_size):
                # Проверяем, не нужно ли прервать
                with self._busy_lock:
                    if self._pending_text or process.poll() is not None:
                        break
                
                chunk = audio_bytes[i:i + chunk_size]
                try:
                    process.stdin.write(chunk)
                    process.stdin.flush()  # Принудительная отправка для меньшей latency
                except BrokenPipeError:
                    break
            
            process.stdin.close()
            
            # Ждем завершения
            return_code = process.wait()
            if return_code != 0:
                stderr_output = process.stderr.read().decode() if process.stderr else "no stderr"
                self.get_logger().error(f"aplay завершился с кодом {return_code}, stderr: {stderr_output}")
            else:
                self.get_logger().info("✓ Аудио воспроизведено успешно")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения: {e}")
        finally:
            # Очищаем ссылку на процесс
            with self._busy_lock:
                self._current_process = None
                # Прерываем процесс, если он еще работает и есть новое сообщение
                if process and process.poll() is None and self._pending_text:
                    try:
                        process.terminate()
                        process.wait(timeout=0.5)
                    except (subprocess.TimeoutExpired, AttributeError):
                        try:
                            process.kill()
                        except:
                            pass
            
            # ВКЛЮЧАЕМ МИКРОФОН ОБРАТНО С ЗАДЕРЖКОЙ (предотвращение эхо)
            time.sleep(0.1)
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
        node = SileroTTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

