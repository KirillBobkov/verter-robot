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
class SpeechToTextTransducerConfig:
    """Конфигурация Speech-to-Text распознавателя с Transducer моделью"""
    MODEL_DIR: str = 'sherpa-onnx-nemo-transducer-giga-am-v2-russian-2025-04-19'
    ENCODER_NAME: str = 'encoder.int8.onnx'
    DECODER_NAME: str = 'decoder.onnx'
    JOINER_NAME: str = 'joiner.onnx'
    TOKENS_NAME: str = 'tokens.txt'
    VAD_MODEL: str = 'silero_vad.onnx'
    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 1600  # 100ms чанки
    CHANNELS: int = 1
    VAD_THRESHOLD: float = 0.5
    SILENCE_CHUNKS: int = 4  # 0.4 сек паузы
    PRE_BUFFER_CHUNKS: int = 5  # 0.5 сек предыстории
    NUM_THREADS: int = 4  # 4 потока = все ядра Cortex-A72
    MAX_SYMBOLS_PER_STEP: int = 3  # ограничение символов за шаг (против зацикливания)
    AUDIO_LATENCY: float = 0.2


class SpeechToTextTransducerNode(Node):
    """ROS2 узел для преобразования речи в текст через Sherpa-ONNX Transducer"""
    
    def __init__(self) -> None:
        super().__init__('speech_to_text_transducer_node')
        
        # Конфигурация
        self.config = SpeechToTextTransducerConfig()
        
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
        
        self.get_logger().info("🎤 SpeechToTextTransducerNode запущен (Sherpa-ONNX Transducer - ОПТИМИЗИРОВАНО)")
        self.get_logger().info(f"   Sample rate: {self.config.SAMPLE_RATE}Hz")
        self.get_logger().info(f"   Block size: {self.config.BLOCK_SIZE} ({self.config.BLOCK_SIZE/self.config.SAMPLE_RATE*1000:.0f}ms)")
        self.get_logger().info(f"   VAD threshold: {self.config.VAD_THRESHOLD}")
        pause_ms = self.config.SILENCE_CHUNKS * self.config.BLOCK_SIZE / self.config.SAMPLE_RATE * 1000
        self.get_logger().info(f"   Pause trigger: {pause_ms:.0f}ms ⚡")
        self.get_logger().info(f"   ONNX threads: {self.config.NUM_THREADS}")
        self.get_logger().info(f"   Max symbols per step: {self.config.MAX_SYMBOLS_PER_STEP}")
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
        """Загрузка моделей Transducer ASR и VAD"""
        # Определяем пути
        encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path = self._resolve_model_paths()
        
        if not all([encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path]):
            raise RuntimeError("Не найдены файлы моделей Sherpa-ONNX Transducer")
        
        # Загрузка Encoder модели
        self.get_logger().info("🔄 Загрузка Encoder модели...")
        encoder_opts = ort.SessionOptions()
        encoder_opts.inter_op_num_threads = self.config.NUM_THREADS
        encoder_opts.intra_op_num_threads = self.config.NUM_THREADS
        encoder_opts.execution_mode = ort.ExecutionMode.ORT_PARALLEL
        encoder_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        encoder_opts.enable_cpu_mem_arena = True
        encoder_opts.enable_mem_pattern = True
        
        self.encoder = ort.InferenceSession(
            encoder_path,
            sess_options=encoder_opts,
            providers=[('CPUExecutionProvider', {
                'arena_extend_strategy': 'kSameAsRequested',
            })],
        )
        
        # Получаем метаданные для decoder state
        meta = self.encoder.get_modelmeta().custom_metadata_map
        self.pred_rnn_layers = int(meta["pred_rnn_layers"])
        self.pred_hidden = int(meta["pred_hidden"])
        self.get_logger().info(f"✓ Encoder модель загружена ({self.config.NUM_THREADS} потоков)")
        
        # Загрузка Decoder модели
        self.get_logger().info("🔄 Загрузка Decoder модели...")
        decoder_opts = ort.SessionOptions()
        decoder_opts.inter_op_num_threads = self.config.NUM_THREADS
        decoder_opts.intra_op_num_threads = self.config.NUM_THREADS
        decoder_opts.execution_mode = ort.ExecutionMode.ORT_PARALLEL
        decoder_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        decoder_opts.enable_cpu_mem_arena = True
        decoder_opts.enable_mem_pattern = True
        
        self.decoder = ort.InferenceSession(
            decoder_path,
            sess_options=decoder_opts,
            providers=[('CPUExecutionProvider', {
                'arena_extend_strategy': 'kSameAsRequested',
            })],
        )
        self.get_logger().info(f"✓ Decoder модель загружена")
        
        # Загрузка Joiner модели
        self.get_logger().info("🔄 Загрузка Joiner модели...")
        joiner_opts = ort.SessionOptions()
        joiner_opts.inter_op_num_threads = self.config.NUM_THREADS
        joiner_opts.intra_op_num_threads = self.config.NUM_THREADS
        joiner_opts.execution_mode = ort.ExecutionMode.ORT_PARALLEL
        joiner_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        joiner_opts.enable_cpu_mem_arena = True
        joiner_opts.enable_mem_pattern = True
        
        self.joiner = ort.InferenceSession(
            joiner_path,
            sess_options=joiner_opts,
            providers=[('CPUExecutionProvider', {
                'arena_extend_strategy': 'kSameAsRequested',
            })],
        )
        self.get_logger().info(f"✓ Joiner модель загружена")
        
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
        self.encoder_input_names = [inp.name for inp in self.encoder.get_inputs()]
        self.encoder_output_names = [out.name for out in self.encoder.get_outputs()]
        self.decoder_input_names = [inp.name for inp in self.decoder.get_inputs()]
        self.decoder_output_names = [out.name for out in self.decoder.get_outputs()]
        self.joiner_input_names = [inp.name for inp in self.joiner.get_inputs()]
        self.joiner_output_names = [out.name for out in self.joiner.get_outputs()]
        self.get_logger().info("✓ Кэш input/output names создан")
        
        # Создание fbank экстрактора
        self.fbank_opts = self._create_fbank_options()
    
    def _resolve_model_paths(self) -> tuple[Optional[str], Optional[str], Optional[str], Optional[str], Optional[str]]:
        """Определение путей к моделям"""
        try:
            package_share = get_package_share_directory('verter_admin')
            model_dir = os.path.join(package_share, self.config.MODEL_DIR)
            
            encoder_path = os.path.join(model_dir, self.config.ENCODER_NAME)
            decoder_path = os.path.join(model_dir, self.config.DECODER_NAME)
            joiner_path = os.path.join(model_dir, self.config.JOINER_NAME)
            tokens_path = os.path.join(model_dir, self.config.TOKENS_NAME)
            vad_model_path = os.path.join(package_share, self.config.VAD_MODEL)
            
            # Проверяем существование
            if not all(os.path.exists(p) for p in [encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path]):
                self.get_logger().error(f"Не найдены файлы в {model_dir}")
                return None, None, None, None, None
            
            return encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска моделей: {e}")
            return None, None, None, None, None
    
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
    
    def _get_decoder_init_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Получить начальное состояние декодера"""
        batch_size = 1
        state0 = np.zeros((self.pred_rnn_layers, batch_size, self.pred_hidden), dtype=np.float32)
        state1 = np.zeros((self.pred_rnn_layers, batch_size, self.pred_hidden), dtype=np.float32)
        return state0, state1
    
    def _start_audio_capture(self) -> None:
        """Запуск захвата аудио в отдельном потоке"""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        self.get_logger().info("✓ Захват аудио запущен")
    
    def _audio_capture_loop(self) -> None:
        """Основной цикл захвата аудио"""
        try:
            # Пробуем без latency
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
                # Конец фразы - запускаем распознавание АСИНХРОННО
                # Копируем буфер чтобы не блокировать audio callback
                audio_to_recognize = list(self.audio_buffer)
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
        """VAD предсказание вероятности речи"""
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
                
                # Добавляем padding в конец (для transducer)
                tail_padding = np.zeros(int(self.config.SAMPLE_RATE * 0.5), dtype=np.float32)
                full_audio = np.concatenate([full_audio, tail_padding])
                
                # Извлекаем признаки
                t1 = time.time()
                fbank = knf.OnlineFbank(self.fbank_opts)
                features = self._compute_features(full_audio, fbank)
                feature_time = time.time() - t1
                
                # Распознавание через Transducer модель
                t2 = time.time()
                text = self._recognize_transducer(features)
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
    
    def _run_encoder(self, features: np.ndarray) -> np.ndarray:
        """Запуск encoder модели"""
        # Преобразование: (T, C) -> (1, C, T)
        x = np.expand_dims(features.T, axis=0).astype(np.float32)
        x_lens = np.array([x.shape[-1]], dtype=np.int64)
        
        encoder_out, out_len = self.encoder.run(
            self.encoder_output_names[:2],
            {
                self.encoder_input_names[0]: x,
                self.encoder_input_names[1]: x_lens,
            },
        )
        return encoder_out
    
    def _run_decoder(self, token: int, state0: np.ndarray, state1: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Запуск decoder модели"""
        target = np.array([[token]], dtype=np.int32)
        target_len = np.array([1], dtype=np.int32)
        
        outputs = self.decoder.run(
            self.decoder_output_names,
            {
                self.decoder_input_names[0]: target,
                self.decoder_input_names[1]: target_len,
                self.decoder_input_names[2]: state0,
                self.decoder_input_names[3]: state1,
            },
        )
        # Возвращаем: decoder_out, state0_next, state1_next
        return outputs[0], outputs[2], outputs[3]
    
    def _run_joiner(self, encoder_out: np.ndarray, decoder_out: np.ndarray) -> np.ndarray:
        """Запуск joiner модели"""
        logit = self.joiner.run(
            self.joiner_output_names[:1],
            {
                self.joiner_input_names[0]: encoder_out,
                self.joiner_input_names[1]: decoder_out,
            },
        )[0]
        return logit
    
    def _recognize_transducer(self, features: np.ndarray) -> str:
        """Transducer распознавание - ОПТИМИЗИРОВАНО"""
        ans = []
        
        # Начальное состояние декодера
        state0, state1 = self._get_decoder_init_state()
        decoder_out, state0, state1 = self._run_decoder(self.blank_id, state0, state1)
        
        # Прогоняем через encoder один раз
        encoder_out = self._run_encoder(features)
        T = encoder_out.shape[2]
        
        # Greedy декодирование с ограничением символов за шаг
        for t in range(T):
            encoder_out_t = encoder_out[:, :, t:t+1]
            
            # Ограничение на количество символов за один временной шаг
            symbols_emitted = 0
            while symbols_emitted < self.config.MAX_SYMBOLS_PER_STEP:
                logits = self._run_joiner(encoder_out_t, decoder_out)
                logits_squeezed = logits.squeeze()
                idx = int(np.argmax(logits_squeezed))
                
                # Проверка границ токена
                if idx < 0 or idx > self.max_token_id:
                    break
                
                if idx == self.blank_id:
                    break
                
                ans.append(idx)
                decoder_out, state0, state1 = self._run_decoder(idx, state0, state1)
                symbols_emitted += 1
        
        # Преобразуем токены в текст
        if not ans:
            return ""
        
        tokens = [self.id2token.get(i, "") for i in ans]
        text = "".join(tokens).replace("▁", " ").strip()
        return text
    
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
            self.encoder = None
            self.decoder = None
            self.joiner = None
            self.vad_model = None
        
        try:
            sd.stop()
        except Exception:
            pass
    
    def shutdown(self) -> None:
        """Корректное завершение работы"""
        try:
            self.get_logger().info("🔄 Завершение работы SpeechToTextTransducerNode...")
            self.shutdown_event.set()
            self._cleanup_resources()
            self.get_logger().info("✅ SpeechToTextTransducerNode завершен")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения: {e}")


def main(args=None) -> None:
    """Главная функция запуска"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechToTextTransducerNode()
        print("🎤 SPEECH-TO-TEXT NODE (Sherpa-ONNX Transducer)")
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

