#!/usr/bin/env python3
import os
import threading
import time
from typing import Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import kaldi_native_fbank as knf
import onnxruntime as ort
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool


@dataclass
class SpeechToTextConfig:
    """Конфигурация Speech-to-Text распознавателя"""
    MODEL_DIR: str = 'sherpa-onnx-nemo-ctc-giga-am-v2-russian-2025-04-19'
    MODEL_NAME: str = 'model.int8.onnx'
    TOKENS_NAME: str = 'tokens.txt'
    VAD_MODEL: str = 'silero_vad.onnx'
    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 1600  # 100ms чанки
    CHANNELS: int = 1
    VAD_THRESHOLD: float = 0.5
    SILENCE_CHUNKS: int = 6  # 0.6 сек паузы
    PRE_BUFFER_CHUNKS: int = 5  # 0.5 сек предыстории
    NUM_THREADS: int = 4  # 4 потока = все ядра Cortex-A72
    AUDIO_LATENCY: float = 0.2


class SpeechToTextNode(Node):
    """ROS2 узел для преобразования речи в текст через Sherpa-ONNX CTC"""
    
    def __init__(self) -> None:
        super().__init__('speech_to_text_node')
        
        # Конфигурация
        self.config = SpeechToTextConfig()
        
        # Флаг активности (для остановки во время TTS)
        self.is_active = True
        self.active_lock = threading.Lock()
        
        # Потокобезопасность
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()
        
        # Буферы для VAD и накопления аудио
        self.audio_buffer = []
        self.pre_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        
        # VAD состояния
        self.vad_h = np.zeros((2, 1, 64), dtype=np.float32)
        self.vad_c = np.zeros((2, 1, 64), dtype=np.float32)
        
        # Инициализация
        self._setup_ros_communication()
        self._setup_audio_system()
        self._load_models()
        self._start_audio_capture()
        
        self.get_logger().info("🎤 SpeechToTextNode запущен (Sherpa-ONNX CTC - ОПТИМИЗИРОВАНО)")
        self.get_logger().info(f"   Sample rate: {self.config.SAMPLE_RATE}Hz")
        self.get_logger().info(f"   Block size: {self.config.BLOCK_SIZE} ({self.config.BLOCK_SIZE/self.config.SAMPLE_RATE*1000:.0f}ms)")
        self.get_logger().info(f"   VAD threshold: {self.config.VAD_THRESHOLD}")
        pause_ms = self.config.SILENCE_CHUNKS * self.config.BLOCK_SIZE / self.config.SAMPLE_RATE * 1000
        self.get_logger().info(f"   Pause trigger: {pause_ms:.0f}ms ⚡")
        self.get_logger().info(f"   ONNX threads: {self.config.NUM_THREADS}")
        self.get_logger().info("   🔀 Асинхронное распознавание + оптимизации (pre-alloc, кэш)")
    
    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 коммуникации"""
        # Publisher для распознанного текста
        self.recognized_text_pub = self.create_publisher(
            String, 
            'recognized_text', 
            10
        )
        
        # Subscribers для управления активностью
        self.create_subscription(
            Bool,
            'speech_control',
            self._handle_activation,
            10
        )
        self.create_subscription(
            Bool,
            'tts_control',
            self._handle_activation,
            10
        )
    
    def _setup_audio_system(self) -> None:
        """Настройка аудио системы"""
        self.audio_device = self._find_audio_device()
    
    def _find_audio_device(self) -> Optional[int]:
        """Поиск подходящего аудио устройства"""
        devices = sd.query_devices()
        
        # Приоритет: ReSpeaker → устройство 1 → первое доступное
        for i, device in enumerate(devices):
            if any(name in device['name'] for name in ['ReSpeaker', 'ArrayUAC10']):
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ Найден ReSpeaker: {device['name']}")
                    return i
        
        # Fallback стратегия
        fallback_devices = [1] + list(range(len(devices)))
        for i in fallback_devices:
            if i < len(devices) and devices[i]['max_input_channels'] > 0:
                self.get_logger().info(f"✓ Используется устройство {i}: {devices[i]['name']}")
                return i
        
        self.get_logger().warn("⚠️ Входные устройства не найдены, используется дефолт")
        return None
    
    def _load_models(self) -> None:
        """Загрузка моделей ASR и VAD"""
        # Определяем пути
        asr_model_path, tokens_path, vad_model_path = self._resolve_model_paths()
        
        if not all([asr_model_path, tokens_path, vad_model_path]):
            raise RuntimeError("Не найдены файлы моделей Sherpa-ONNX")
        
        # Загрузка ASR модели с оптимизацией для RPi4
        self.get_logger().info("🔄 Загрузка ASR модели...")
        session_opts = ort.SessionOptions()
        session_opts.intra_op_num_threads = self.config.NUM_THREADS
        session_opts.inter_op_num_threads = 1
        session_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED
        session_opts.enable_cpu_mem_arena = True
        session_opts.enable_mem_pattern = True
        session_opts.add_session_config_entry("session.intra_op.allow_spinning", "1")
        session_opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
        
        # CPU provider с оптимизациями для ARM
        providers = [
            ('CPUExecutionProvider', {
                'arena_extend_strategy': 'kSameAsRequested',
            })
        ]
        
        self.asr_model = ort.InferenceSession(
            asr_model_path,
            sess_options=session_opts,
            providers=providers,
        )
        self.get_logger().info(f"✓ ASR модель загружена ({self.config.NUM_THREADS} потоков, extended opt)")
        
        # Загрузка VAD модели
        self.get_logger().info("🔄 Загрузка VAD модели...")
        vad_opts = ort.SessionOptions()
        vad_opts.intra_op_num_threads = 1
        vad_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        self.vad_model = ort.InferenceSession(
            vad_model_path,
            sess_options=vad_opts,
            providers=["CPUExecutionProvider"],
        )
        self.get_logger().info(f"✓ VAD модель загружена")
        
        # Загрузка токенов
        self.id2token = self._load_tokens(tokens_path)
        self.blank_id = len(self.id2token) - 1
        self.max_token_id = max(self.id2token.keys()) if self.id2token else self.blank_id
        self.get_logger().info(f"✓ Загружено токенов: {len(self.id2token)}")
        
        # КЭШ для input/output names (критично для производительности!)
        self.asr_input_names = [inp.name for inp in self.asr_model.get_inputs()]
        self.asr_output_names = [out.name for out in self.asr_model.get_outputs()]
        self.get_logger().info("✓ Кэш input/output names создан")
        
        # Создание fbank экстрактора
        self.fbank_opts = self._create_fbank_options()
    
    def _resolve_model_paths(self) -> tuple[Optional[str], Optional[str], Optional[str]]:
        """Определение путей к моделям"""
        try:
            package_share = get_package_share_directory('verter_admin')
            model_dir = os.path.join(package_share, self.config.MODEL_DIR)
            
            asr_model = os.path.join(model_dir, self.config.MODEL_NAME)
            tokens = os.path.join(model_dir, self.config.TOKENS_NAME)
            vad_model = os.path.join(package_share, self.config.VAD_MODEL)
            
            # Проверяем существование
            if not all(os.path.exists(p) for p in [asr_model, tokens, vad_model]):
                self.get_logger().error(f"Не найдены файлы в {model_dir}")
                return None, None, None
            
            return asr_model, tokens, vad_model
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска моделей: {e}")
            return None, None, None
    
    def _load_tokens(self, tokens_path: str) -> dict:
        """Загрузка словаря токенов"""
        id2token = {}
        with open(tokens_path, encoding="utf-8") as f:
            for line in f:
                fields = line.split()
                if len(fields) == 1:
                    id2token[int(fields[0])] = " "
                else:
                    token, idx = fields
                    id2token[int(idx)] = token
        return id2token
    
    def _create_fbank_options(self) -> knf.FbankOptions:
        """Создание опций для fbank экстрактора"""
        opts = knf.FbankOptions()
        opts.frame_opts.dither = 0
        opts.frame_opts.remove_dc_offset = False
        opts.frame_opts.preemph_coeff = 0
        opts.frame_opts.window_type = "hann"
        opts.frame_opts.round_to_power_of_two = False
        opts.mel_opts.low_freq = 0
        opts.mel_opts.high_freq = 8000
        opts.mel_opts.num_bins = 64
        return opts
    
    def _start_audio_capture(self) -> None:
        """Запуск захвата аудио в отдельном потоке"""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        self.get_logger().info("✓ Захват аудио запущен")
    
    def _audio_capture_loop(self) -> None:
        """Основной цикл захвата аудио"""
        try:
            # Пробуем без latency (как в simple_mic_vad.py)
            with sd.InputStream(
                samplerate=self.config.SAMPLE_RATE,
                blocksize=self.config.BLOCK_SIZE,
                device=self.audio_device,
                dtype='float32',
                channels=self.config.CHANNELS,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Аудио поток успешно запущен")
                self.shutdown_event.wait()
        except Exception as e:
            # Если не получилось - пробуем с latency
            self.get_logger().warn(f"Ошибка без latency: {e}")
            self.get_logger().info("Попытка с латентностью...")
            try:
                with sd.InputStream(
                    samplerate=self.config.SAMPLE_RATE,
                    blocksize=self.config.BLOCK_SIZE,
                    device=self.audio_device,
                    dtype='float32',
                    channels=self.config.CHANNELS,
                    latency=self.config.AUDIO_LATENCY,
                    callback=self._audio_callback
                ):
                    self.get_logger().info("✓ Аудио поток запущен с latency")
                    self.shutdown_event.wait()
            except Exception as e2:
                self.get_logger().error(f"Критическая ошибка аудио: {e2}")
    
    def _audio_callback(self, indata, frames, time_info, status) -> None:
        """Callback обработки аудио данных"""
        if self.shutdown_event.is_set():
            return
        
        # Проверяем активность
        with self.active_lock:
            if not self.is_active:
                return
        
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        try:
            audio_chunk = indata.flatten().copy()
            self._process_audio_chunk(audio_chunk)
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио callback: {e}")
    
    def _process_audio_chunk(self, audio_chunk: np.ndarray) -> None:
        """Обработка аудио чанка с VAD"""
        # VAD - определение речи
        speech_prob = self._vad_predict(audio_chunk)
        is_speech = speech_prob > self.config.VAD_THRESHOLD
        
        if is_speech:
            if not self.is_speaking:
                self.is_speaking = True
                self.audio_buffer = list(self.pre_buffer)
                self.pre_buffer = []
            self.audio_buffer.append(audio_chunk)
            self.silence_counter = 0
            
        elif self.is_speaking:
            # Продолжаем накапливать после речи (для ловли конца фразы)
            self.audio_buffer.append(audio_chunk)
            self.silence_counter += 1
            
            if self.silence_counter >= self.config.SILENCE_CHUNKS:
                # Конец фразы — отрезаем накопленную тишину в конце перед распознаванием
                chunks_to_drop = min(self.config.SILENCE_CHUNKS, len(self.audio_buffer))
                trimmed_chunks = self.audio_buffer[:-chunks_to_drop] if chunks_to_drop > 0 else self.audio_buffer
                
                # Запускаем распознавание АСИНХРОННО (копируем буфер чтобы не блокировать callback)
                audio_to_recognize = list(trimmed_chunks)
                threading.Thread(
                    target=self._recognize_accumulated_audio,
                    args=(audio_to_recognize,),
                    daemon=True
                ).start()
                
                self.is_speaking = False
                self.silence_counter = 0
                self.audio_buffer = []
        else:
            # Нет речи - сохраняем в предбуфер
            self.pre_buffer.append(audio_chunk)
            if len(self.pre_buffer) > self.config.PRE_BUFFER_CHUNKS:
                self.pre_buffer.pop(0)
    
    def _vad_predict(self, audio_chunk: np.ndarray) -> float:
        """VAD предсказание вероятности речи - как в simple_mic_vad.py"""
        # Silero VAD требует 512 сэмплов для 16kHz
        if len(audio_chunk) < 512:
            audio_chunk = np.pad(audio_chunk, (0, 512 - len(audio_chunk)))
        audio_chunk = audio_chunk[:512].astype(np.float32)
        
        ort_inputs = {
            'x': audio_chunk.reshape(1, -1),
            'h': self.vad_h,
            'c': self.vad_c
        }
        
        ort_outs = self.vad_model.run(None, ort_inputs)
        prob = ort_outs[0][0][0]
        self.vad_h = ort_outs[1]
        self.vad_c = ort_outs[2]
        
        return float(prob)
    
    def _recognize_accumulated_audio(self, audio_buffer: list) -> None:
        """Распознавание накопленного аудио (в отдельном потоке!)"""
        if not audio_buffer:
            return
        
        start_time = time.time()
        with self.recognition_lock:
            try:
                # Объединяем все чанки
                full_audio = np.concatenate(audio_buffer)
                
                # Извлекаем признаки
                t1 = time.time()
                fbank = knf.OnlineFbank(self.fbank_opts)
                features = self._compute_features(full_audio, fbank)
                feature_time = time.time() - t1
                
                # Распознавание через CTC модель
                t2 = time.time()
                text = self._recognize_ctc(features)
                inference_time = time.time() - t2
                
                total_time = time.time() - start_time
                
                if text:
                    self._publish_recognized_text(text)
                    self.get_logger().info(
                        f"⏱️  Timing: feature={feature_time*1000:.0f}ms, "
                        f"inference={inference_time*1000:.0f}ms, "
                        f"total={total_time*1000:.0f}ms"
                    )
            except Exception as e:
                self.get_logger().error(f"Ошибка распознавания: {e}")
    
    def _compute_features(self, audio: np.ndarray, fbank: knf.OnlineFbank) -> np.ndarray:
        """Извлечение fbank признаков - ОПТИМИЗИРОВАНО с pre-allocation"""
        fbank.accept_waveform(self.config.SAMPLE_RATE, audio)
        num_frames = fbank.num_frames_ready
        
        if num_frames == 0:
            return np.zeros((1, 64), dtype=np.float32)
        
        # PRE-ALLOCATE массив (вместо list.append) - критично для производительности!
        features = np.empty((num_frames, 64), dtype=np.float32)
        for i in range(num_frames):
            features[i] = fbank.get_frame(i)  # Прямое присваивание без промежуточных массивов
        
        return features
    
    def _recognize_ctc(self, features: np.ndarray) -> str:
        """CTC распознавание - ОПТИМИЗИРОВАНО с кэшированными именами"""
        # Оптимизированное преобразование: (T, 64) -> (1, 64, T)
        # Используем expand_dims вместо newaxis (более явно и эффективно)
        if features.ndim == 2:
            x = np.expand_dims(features.T, axis=0).astype(np.float32)
        else:
            x = features.astype(np.float32)
        
        x_lens = np.array([x.shape[-1]], dtype=np.int64)
        
        # Inference с КЭШИРОВАННЫМИ именами (критично для производительности!)
        log_probs = self.asr_model.run(
            self.asr_output_names[:1],  # Используем кэш!
            {
                self.asr_input_names[0]: x,      # Используем кэш!
                self.asr_input_names[1]: x_lens, # Используем кэш!
            },
        )[0]
        
        # CTC декодирование (collapse repeats) - pure numpy
        ids = np.argmax(log_probs[0], axis=1)
        
        # Оптимизированное декодирование: фильтруем blank и повторы за один проход
        tokens = []
        prev = -1
        for i in ids:
            # Проверка границ токена (защита от ошибок)
            if i < 0 or i > self.max_token_id:
                continue
            if i != self.blank_id and i != prev:
                token = self.id2token.get(int(i), "")
                if token:  # Пропускаем пустые токены
                    tokens.append(token)
            prev = i
        
        return "".join(tokens).strip()
    
    def _publish_recognized_text(self, text: str) -> None:
        """Публикация распознанного текста"""
        # Двойная проверка активности
        with self.active_lock:
            if not self.is_active:
                self.get_logger().debug(f"🔇 Пропущено (деактивирован): '{text}'")
                return
        
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        self.get_logger().info(f"🎤 Распознано: '{text}'")
    
    def _handle_activation(self, msg: Bool) -> None:
        """Обработка команды активации/деактивации"""
        with self.active_lock:
            self.is_active = msg.data
        
        # Сбрасываем буферы при деактивации
        if not msg.data:
            self.audio_buffer = []
            self.pre_buffer = []
            self.is_speaking = False
            self.silence_counter = 0
        
        status = "включен" if msg.data else "выключен (TTS активен)"
        self.get_logger().info(f"📡 SpeechToText {status}")
    
    def _cleanup_resources(self) -> None:
        """Очистка ресурсов"""
        with self.recognition_lock:
            self.asr_model = None
            self.vad_model = None
        
        try:
            sd.stop()
        except Exception:
            pass
    
    def shutdown(self) -> None:
        """Корректное завершение работы"""
        try:
            self.get_logger().info("🔄 Завершение работы SpeechToTextNode...")
            self.shutdown_event.set()
            self._cleanup_resources()
            self.get_logger().info("✅ SpeechToTextNode завершен")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения: {e}")


def main(args=None) -> None:
    """Главная функция запуска"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechToTextNode()
        print("🎤 SPEECH-TO-TEXT NODE (Sherpa-ONNX CTC)")
        print("   Публикация в топик: recognized_text")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🔄 Завершение по Ctrl+C...")
    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Система завершена")


if __name__ == '__main__':
    main()
