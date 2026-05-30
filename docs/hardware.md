# Аппаратная часть

## Исследование бортового компьютера для Verter

Дата: 2026-05-03.

### Краткий вывод

Лучший базовый выбор для текущего робота: **NVIDIA Jetson Orin Nano Super Developer Kit 8GB + NVMe SSD**.

Причина: текущий стек Verter уже ориентирован на Linux/ROS2 на Jetson, использует ROS2 Humble, Nav2, SLAM Toolbox, micro-ROS, аудио, ONNX/PyTorch/OpenCV и потенциально RealSense. Для такого набора Jetson Orin Nano Super дает лучший баланс: достаточная CPU-производительность для навигации, CUDA/TensorRT запас для будущего computer vision/AI, компактный форм-фактор, 7-25 W потребления, 4 USB-A порта, USB-C, NVMe и официальную Ubuntu 22.04/JetPack 6 поддержку.

Если планируется тяжелое локальное зрение, несколько камер, локальный LLM или полноценный мультимодальный ассистент, следующий уровень: **Jetson Orin NX 16GB на совместимом carrier board**. Для текущего LiDAR+Nav2+голосового ассистента Orin NX 16GB не обязателен.

### Требования текущего проекта

По launch-файлам и зависимостям проекта бортовой компьютер должен уверенно тянуть:

| Подсистема | Что запускается | Нагрузка |
|---|---|---|
| Базовая навигация | ROS2 Humble, Nav2, AMCL, planner/controller/behavior servers, waypoint follower | CPU/RAM, real-time стабильность |
| Карта и автономное исследование | SLAM Toolbox, Explore Lite, laser_filters | CPU/RAM, стабильный `/scan` |
| Низкий уровень | 2 x `micro_ros_agent` на 921600 baud для ESP32 chassis/IMU | стабильный USB serial |
| Сенсоры | RPLiDAR A1, ультразвук/range converter, IMU, odometry, EKF | USB/serial, низкая задержка |
| Голос | Sherpa/ONNX STT, Silero VAD, Piper TTS, ReSpeaker USB, DOA | CPU, иногда GPU/NPU полезны |
| Web UI | rosbridge/web server + frontend | небольшая CPU/RAM нагрузка |
| Потенциальное зрение | `realsense2_camera`, OpenCV, PyTorch, ONNX Runtime | USB3 + GPU/AI accelerator желательно |

Минимально разумные требования:

| Параметр | Минимум | Рекомендуется |
|---|---:|---:|
| RAM | 8 GB | 8-16 GB |
| Накопитель | 64 GB microSD/SSD | 256-512 GB NVMe SSD |
| USB | 4 порта, из них хотя бы 1 USB3 | 4+ USB3 + powered hub для сенсоров |
| Питание компьютера | 15-25 W запас | отдельный стабильный DC-DC с запасом 2x |
| ОС | Ubuntu 22.04 | Ubuntu 22.04 + ROS2 Humble |
| AI ускорение | не обязательно для LiDAR-only | CUDA/TensorRT для CV/ASR/LLM |

### Сравнение вариантов

| Вариант | Оценка для Verter | Плюсы | Минусы | Когда выбирать |
|---|---|---|---|---|
| **Jetson Orin Nano Super Dev Kit 8GB** | **Лучший основной вариант** | 67 INT8 TOPS, 8GB LPDDR5, 7-25 W, CUDA/TensorRT, 4 x USB 3.2 Type-A, NVMe, CSI, Ubuntu 22.04/JetPack | 8GB RAM может стать лимитом для LLM/тяжелого CV; ARM иногда требует сборки пакетов | Текущий робот: Nav2 + SLAM + STT/TTS + будущая камера/AI |
| **Jetson Orin NX 16GB** | Лучший с запасом | До 157 TOPS, 16GB RAM, больше запас под vision/LLM/multi-camera | Дороже, выше требования к питанию/охлаждению, нужен carrier | Если нужна серьезная локальная AI-нагрузка |
| **Intel N100/N150/N305 mini PC** | Хороший x86-вариант без CUDA | Простая Ubuntu amd64, дешевые mini-PC, много RAM/SSD, быстрый CPU для ROS/Nav2 | Нет NVIDIA CUDA/TensorRT; слабое edge-AI; часто бытовое питание/USB без rugged-гарантий | Если AI/CV почти нет, важнее x86-совместимость |
| **Raspberry Pi 5 8/16GB** | Не рекомендован как основной | Дешевый, компактный, ROS2 arm64 возможен, низкое потребление | Нет CUDA, слабее под ONNX/STT/CV, USB/питание требуют аккуратности, NVMe через HAT | Только бюджетный LiDAR-only прототип |
| **Jetson AGX Orin / Jetson Thor** | Перебор для текущей версии | Очень большой AI запас | Цена, потребление, размеры | Только для исследовательской платформы с тяжелыми моделями |

### Рекомендуемая конфигурация

Для текущей версии Verter:

| Компонент | Рекомендация |
|---|---|
| Бортовой компьютер | NVIDIA Jetson Orin Nano Super Developer Kit 8GB |
| ОС | JetPack 6.2.x / Jetson Linux 36.x, Ubuntu 22.04 based rootfs |
| ROS | ROS2 Humble, потому что официальные deb-пакеты есть для Ubuntu 22.04 amd64/arm64 |
| Накопитель | NVMe SSD 256 GB минимум, лучше 512 GB; microSD использовать только для первичной установки/резерва |
| Питание Jetson | Отдельный DC-DC на 19 V или стабильный вход согласно carrier board; запас по мощности не меньше 50 W |
| USB | RPLiDAR, ReSpeaker, ESP32 chassis, ESP32 IMU лучше развести через качественный powered USB hub или отдельные USB-линии |
| Охлаждение | Активное, штатный радиатор/вентилятор обязателен; для закрытого корпуса нужен airflow |
| Камера | Если добавляется RealSense D435i, подключать в USB3 и учитывать отдельный бюджет питания/кабеля |

### Почему не Raspberry Pi 5

Raspberry Pi 5 уже достаточно быстрый для легкого ROS2, но для Verter одновременно идут SLAM/Nav2, аудио pipeline, ONNX Runtime, web, micro-ROS agents и потенциально камера. Без CUDA/TensorRT запас по AI-нагрузке хуже. Pi 5 можно использовать как дешевый прототип, но не как целевой бортовой компьютер для робота с голосом и будущим зрением.

### Почему не обычный Intel mini-PC как основной выбор

Intel N100/N305 mini-PC удобен для ROS2 на Ubuntu amd64 и может хорошо тянуть LiDAR-only Nav2/SLAM. Но текущий проект уже содержит `torch`, `opencv-python`, `onnxruntime`, `realsense2_camera`, STT/TTS и AI assistant. Как только понадобится детекция объектов, depth-camera processing или локальные модели, Jetson выигрывает за счет CUDA/TensorRT и экосистемы edge robotics. Intel mini-PC имеет смысл, если принципиально нужен x86 и AI будет вынесен в облако/внешний ускоритель.

### Риски и практические замечания

- Не запускать систему с microSD как основным накопителем: SLAM-карты, логи, модели и сборки ROS быстро упираются в скорость/износ. Нужен NVMe.
- USB-стабильность важнее количества TOPS: RPLiDAR, два ESP32 и ReSpeaker должны получать стабильное питание и не отваливаться при старте моторов.
- Питание компьютера должно быть отделено от шумной моторной линии через нормальный DC-DC, фильтрацию и общий ground design.
- Если используется RealSense, нужен короткий качественный USB3 кабель и отдельный запас по питанию USB.
- Для ONNX Runtime на Jetson стоит отдельно проверить, какие модели выгоднее гонять через CPU, CUDA или TensorRT. VAD часто лучше оставить на CPU, тяжелую ASR/CV модель переводить на GPU/TensorRT.

### Источники

- NVIDIA Jetson Orin Nano Super Developer Kit: https://www.nvidia.com/en-in/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/
- NVIDIA Jetson Orin family specs: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/
- NVIDIA Jetson Orin Nano Developer Kit hardware specs: https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/hardware_spec.html
- NVIDIA JetPack 6.2.2 / Ubuntu 22.04 based rootfs: https://developer.nvidia.com/embedded/jetpack-sdk-622
- ROS2 Humble Ubuntu packages: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Raspberry Pi 5 specs: https://www.raspberrypi.com/products/raspberry-pi-5/
- Intel Processor N100 specs: https://www.intel.com/content/www/us/en/products/sku/231803/intel-processor-n100-6m-cache-up-to-3-40-ghz.html
- RPLIDAR A1 specs: https://www.slamtec.com/en/lidar/a1spec
- Intel RealSense D435i specs: https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i.html
