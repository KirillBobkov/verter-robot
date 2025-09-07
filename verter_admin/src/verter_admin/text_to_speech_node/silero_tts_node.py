#!/usr/bin/env python3

import os
import threading
import subprocess
import io
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

try:
    from silero_tts.silero_tts import SileroTTS
    import torch
    import torchaudio
except ImportError:
    SileroTTS = None
    torch = None
    torchaudio = None

class SileroTTSNode(Node):
    
    def __init__(self):
        super().__init__('silero_tts_node')
        
        if SileroTTS is None:
            self.get_logger().error("silero-tts не установлен. Установите: pip install silero-tts")
            raise ImportError("silero-tts не найден")
        
        self.language = 'ru'
        self.speaker = 'aidar'
        self.audio_device = "pulse"
        
        self._setup_environment()
        self._initialize_tts()
        
        self.subscription = self.create_subscription(String, 'ai_response', self.text_callback, 10)
        self.recognition_publisher = self.create_publisher(Bool, 'set_recognition_active', 10)
        
        self._busy_lock = threading.Lock()
        self._busy = False
        self._pending_text = None

        self.get_logger().info(f"Silero TTS запущен. Модель: {self.tts.model_id}, Голос: {self.speaker}")
    
    def _setup_environment(self):
        self.env = os.environ.copy()
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")
    
    def _initialize_tts(self):
        model_id = SileroTTS.get_latest_model(self.language)
        self.tts = SileroTTS(model_id=model_id, language=self.language, speaker=self.speaker)
    
    
    def text_callback(self, msg: String):
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
        # Генерируем аудио напрямую в память
        audio_buffer = io.BytesIO()
        
        # Синтез речи в buffer
        self.tts.tts(text.strip(), audio_buffer)
        audio_buffer.seek(0)
        
        # Стриминг через pipe в aplay
        process = subprocess.Popen(
            ["aplay", "-D", self.audio_device, "-q"],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=self.env
        )
        
        # Передаем данные из buffer в stdin aplay
        process.communicate(input=audio_buffer.getvalue())
        self._activate_speech_recognition()
    
    def _activate_speech_recognition(self):
        msg = Bool()
        msg.data = True
        self.recognition_publisher.publish(msg)
    


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
