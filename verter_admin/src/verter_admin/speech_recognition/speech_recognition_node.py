#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import vosk
import sounddevice as sd
import queue
import json
import os
import threading
import time
import usb.core
import usb.util
from ament_index_python.packages import get_package_share_directory

# -*- coding: utf-8 -*-

import sys
import struct
import usb.core
import usb.util

USAGE = """Usage: python {} -h
        -p      show all parameters
        -r      read all parameters
        NAME    get the parameter with the NAME
        NAME VALUE  set the parameter with the NAME and the VALUE
"""



# parameter list
# name: (id, offset, type, max, min , r/w, info)
PARAMETERS = {
    'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients'),
    'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds'),
    'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off'),
    'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AES. 0 = OFF 1 = ON'),
    'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-09, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))'),
    'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status. ', '0 = false (signal detected) ', '1 = true (silence detected)'),
    'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control. ', '0 = OFF ', '1 = ON'),
    'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor. ', '[0 .. 60] dB (default 30dB = 20log10(31.6))'),
    'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-08, 'rw', 'Target power level of the output signal. ', '[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))'),
    'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor. ', '[0 .. 60] dB (default: 0.0dB = 20log10(1.0))'),
    'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'Ramps-up / down time-constant in seconds.'),
    'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON'),
    'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise. min .. max attenuation'),
    'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor of non- stationary noise. min .. max attenuation'),
    'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct and early components). min .. max attenuation'),
    'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components). min .. max attenuation'),
    'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo. min .. max attenuation'),
    'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-Linear echo attenuation.', '0 = OFF', '1 = ON'),
    'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-Linear AEC training mode.', '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2'),
    'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', '0 = false (no speech detected)', '1 = true (speech detected)'),
    'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB Update Decision.', '0 = false (FSB was not updated)', '1 = true (FSB was updated)'),
    'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON'),
    'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)'),
    'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.0)'),
    'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.1)'),
    'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'Set the threshold for voice activity detection.', '[-inf .. 60] dB (default: 3.5dB 20log10(1.5))'),
    # 'KEYWORDDETECT': (20, 0, 'int', 1, 0, 'ro', 'Keyword detected. Current value so needs polling.'),
    'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.')
}


class Tuning:
    TIMEOUT = 100000

    def __init__(self, dev):
        self.dev = dev

    def write(self, name, value):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        if data[5] == 'ro':
            raise ValueError('{} is read-only'.format(name))

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id, payload, self.TIMEOUT)

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self.TIMEOUT)

        response = struct.unpack(b'ii', response.tobytes())

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])

        return result

    def set_vad_threshold(self, db):
        self.write('GAMMAVAD_SR', db)

    def is_voice(self):
        return self.read('VOICEACTIVITY')

    @property
    def direction(self):
        return self.read('DOAANGLE')

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)


def find(vid=0x2886, pid=0x0018):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return

    # configuration = dev.get_active_configuration()

    # interface_number = None
    # for interface in configuration:
    #     interface_number = interface.bInterfaceNumber

    #     if dev.is_kernel_driver_active(interface_number):
    #         dev.detach_kernel_driver(interface_number)

    return Tuning(dev)



def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == '-p':
            print('name\t\t\ttype\tmax\tmin\tr/w\tinfo')
            print('-------------------------------')
            for name in sorted(PARAMETERS.keys()):
                data = PARAMETERS[name]
                print('{:16}\t{}'.format(name, '\t'.join([str(i) for i in data[2:7]])))
                for extra in data[7:]:
                    print('{}{}'.format(' '*60, extra))
        else:
            dev = find()
            if not dev:
                print('No device found')
                sys.exit(1)

            # print('version: {}'.format(dev.version))

            if sys.argv[1] == '-r':
                print('{:24} {}'.format('name', 'value'))
                print('-------------------------------')
                for name in sorted(PARAMETERS.keys()):
                    print('{:24} {}'.format(name, dev.read(name)))
            else:
                name = sys.argv[1].upper()
                if name in PARAMETERS:
                    if len(sys.argv) > 2:
                        dev.write(name, sys.argv[2])
                    
                    print('{}: {}'.format(name, dev.read(name)))
                else:
                    print('{} is not a valid name'.format(name))

            dev.close()
    else:
        print(USAGE.format(sys.argv[0]))

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Инициализация базовых атрибутов
        self.audio_queue_ = queue.Queue(maxsize=50)
        self.shutdown_event_ = threading.Event()
        
        # Настройка компонентов
        self._setup_parameters()
        self._setup_vosk()
        self._setup_doa()
        self._start_audio_capture()
        
        # Запуск обработки
        self.process_timer = self.create_timer(0.05, self.process_audio)
        self.doa_timer = self.create_timer(1.0, self.process_doa)  # Таймер для DOA (1 Гц)
        self.get_logger().info("SpeechRecognitionNode инициализирован успешно")

    def _find_audio_device(self):
        """Автоматический поиск аудио устройства"""
        try:
            devices = sd.query_devices()
            
            # Ищем любое доступное устройство ввода
            for i, device in enumerate(devices):
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"Найдено аудиоустройство: device {i} - {device['name']}")
                    try:
                        sd.check_input_settings(device=i, samplerate=16000, channels=1)
                        self.get_logger().info(f"✓ Устройство {i} поддерживает требуемые параметры")
                        return i
                    except Exception as e:
                        self.get_logger().warn(f"Устройство {i} не поддерживает параметры: {e}")
                        continue
            
            # Fallback на системное устройство по умолчанию
            self.get_logger().warn("Используется устройство по умолчанию")
            return None
            
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска устройства: {e}")
            return None

    def _find_respeaker_device(self):
        """Поиск ReSpeaker устройства"""
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if 'ReSpeaker' in device['name'] or 'ArrayUAC10' in device['name']:
                self.get_logger().info(f"✓ Найден ReSpeaker: device {i} - {device['name']}")
                return i
        
        # Fallback на hw:3,0
        self.get_logger().info("Используется hw:3,0 для ReSpeaker")
        return 3

    def _setup_parameters(self):
        """Настройка локальных параметров"""
        self.model_name_ = 'vosk-model-small-ru-0.22'
        self.samplerate_ = 16000
        self.blocksize_ = 3200
        self.channels_ = 1
        self.device_ = self._find_respeaker_device()
        vosk.SetLogLevel(-1)  # Отключить логи Vosk

    def _setup_vosk(self):
        """Инициализация модели и распознавателя Vosk"""
        model_path = self._resolve_model_path()
        if not model_path:
            raise RuntimeError("Failed to resolve model path")
            
        try:
            self.model_ = vosk.Model(model_path)
            self.recognizer_ = vosk.KaldiRecognizer(self.model_, self.samplerate_)
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
                package_share = get_package_share_directory('verter_admin')
                model_path = os.path.join(package_share, self.model_name_)
            except Exception as e:
                self.get_logger().error(f"Пакет 'verter_admin' не найден: {e}")
                return None
        
        if not os.path.isdir(model_path):
            self.get_logger().error(f"Директория модели не найдена: {model_path}")
            return None
            
        return model_path

    def _setup_doa(self):
        """Инициализация DOA"""
        try:
            # Настройка USB для ReSpeaker
            self.doa_dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if self.doa_dev is None:
                self.get_logger().error("ReSpeaker не найден для DOA")
                self.doa_enabled = False
            else:
                # Класс Tuning уже определён в этом файле
                self.mic_tuning = Tuning(self.doa_dev)
                self.doa_enabled = True
                self.get_logger().info("ReSpeaker для DOA инициализирован успешно")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка инициализации DOA: {e}")
            self.doa_enabled = False

    def _start_audio_capture(self):
        """Запуск потока захвата аудио"""
        self.audio_thread_ = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread_.daemon = True
        self.audio_thread_.start()

    def _audio_capture_loop(self):
        """Основной цикл захвата аудио"""
        try:
            device_info = sd.query_devices(self.device_, 'input') if self.device_ is not None else None
            device_name = device_info['name'] if device_info else 'default'
            self.get_logger().info(f"Используется аудиоустройство: {device_name}")
            
            with sd.RawInputStream(
                samplerate=self.samplerate_,
                blocksize=self.blocksize_,  
                device=self.device_,
                dtype='int16',
                channels=self.channels_,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Захват аудио запущен успешно")
                self.shutdown_event_.wait()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка захвата аудио: {e}")

    def _audio_callback(self, indata, frames, time, status):
        """Callback без отладки"""
        if status:
            self.get_logger().warn(f"Статус аудио: {status}")
        
        if not self.shutdown_event_.is_set():
            self.audio_queue_.put(bytes(indata))

    def process_audio(self):
        """Батчевая обработка без блокировки"""
        if self.shutdown_event_.is_set():
            return
        
        # Обработать все данные за раз
        processed = 0
        while not self.audio_queue_.empty() and processed < 10:  # Лимит на batch
            try:
                data = self.audio_queue_.get_nowait()
                if self.recognizer_.AcceptWaveform(data):
                    result = json.loads(self.recognizer_.Result())
                    text = result.get('text', '').strip()
                    if text:
                        self.get_logger().info(f"Распознано: {text}")
                processed += 1
            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f"Ошибка: {e}")
                break

    def process_doa(self):
        """Обработка и логирование угла DOA"""
        if not hasattr(self, 'doa_enabled') or not self.doa_enabled:
            return
        
        try:
            doa_angle = self.mic_tuning.direction
            # self.get_logger().info(f"DOA угол: {doa_angle} градусов")
        except Exception as e:
            self.get_logger().error(f"Ошибка получения DOA: {e}")

    def shutdown(self):
        """Завершение работы узла"""
        self.shutdown_event_.set()
        
        # Остановка аудио
        try:
            sd.stop()
        except:
            pass
        
        # Очистка USB для DOA
        if hasattr(self, 'doa_dev') and self.doa_dev:
            usb.util.dispose_resources(self.doa_dev)
        
        # Ожидание завершения потока
        if hasattr(self, 'audio_thread_'):
            self.audio_thread_.join(timeout=1.0)
        
        # Очистка очереди
        while not self.audio_queue_.empty():
            try:
                self.audio_queue_.get_nowait()
            except:
                break

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
        print("Узел распознавания речи и DOA запущен. Говорите...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nЗавершение работы...")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()