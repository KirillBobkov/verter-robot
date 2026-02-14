#!/usr/bin/env python3
"""
===================================================================================
ODOMETRY NODE - Нода одометрии для мобильного робота
===================================================================================

ЧТО ТАКОЕ ОДОМЕТРИЯ?
--------------------
Одометрия (от греч. "hodos" - путь, "metron" - мера) - это процесс определения
положения робота в пространстве на основе данных о его движении.

Аналогия из жизни:
Представь, что ты стоишь с закрытыми глазами. Если ты пройдешь 5 шагов вперед
и повернешься налево на 90°, ты можешь вычислить своё новое положение относительно
начальной точки. Именно это делает одометрия для робота.


ЗАЧЕМ ЭТО НУЖНО?
-----------------
1. Навигация: Nav2 нужно знать ГДЕ робот находится, чтобы построить путь к цели
2. Картографирование: Чтобы строить карту, нужно знать откуда робот смотрит
3. Планирование пути: Чтобы объезжать препятствия, нужно знать позицию робота


КАК ЭТО РАБОТАЕТ?
------------------
ОСНОВНОЙ РЕЖИМ (энкодерная одометрия - closed-loop):
1. Получаем данные с энкодеров колёс (шаги AS5600)
2. Вычисляем пройденное расстояние каждым колесом
3. Рассчитываем позицию (x, y, theta) по формулам дифференциального привода
4. Рассчитываем актуальную скорость (vx, vth) из изменения позиции за время
5. Публикуем результат в ROS2 топики

РЕЗЕРВНЫЙ РЕЖИМ (cmd_vel одометрия - open-loop):
1. Если энкодеры не доступны, используем команды скорости
2. ИНТЕГРИРУЕМ скорость по времени: позиция = позиция + скорость * dt
3. Скорость берётся из команд (не из реальных измерений)


ВАЖНЫЕ КОНЦЕПЦИИ ROS2:
----------------------

1. ТОПИКИ (Topics) - это как "каналы связи" между нодами:
   - Одни ноды ПУБЛИКУЮТ (publish) данные в топик
   - Другие ноды ПОДПИСЫВАЮТСЯ (subscribe) на топик и получают данные
   - Аналогия: Как подписка на канал в YouTube

2. СООБЩЕНИЯ (Messages) - структуры данных, которые передаются в топиках:
   - Twist: скорость (linear.x, angular.z)
   - Odometry: позиция и скорость робота
   - Аналогия: Формат письма (адрес, тема, текст)

3. TF (Transform) - система координат в ROS2:
   - Описывает где находится каждая часть робота относительно других частей
   - Например: где колеса относительно центра робота
   - Аналогия: GPS координаты, только для робота

4. СИСТЕМЫ КООРДИНАТ (Frames):
   - odom: Начальная точка робота (откуда он стартовал)
   - base_link: Центр робота (тело робота)
   - map: Глобальная карта мира (добавится позже при использовании SLAM)


ПУБЛИКУЕМЫЕ ТОПИКИ:
-------------------
/odom (nav_msgs/Odometry):
    - Текущая позиция робота (x, y, theta)
    - Текущая скорость робота
    - Аналогия: "Я нахожусь в точке (3м, 2м), смотрю на север, двигаюсь 0.5м/с"

/tf (geometry_msgs/TransformStamped):
    - Трансформация между системами координат odom -> base_link
    - Нужна для Nav2 и всех остальных компонентов ROS2


ПОДПИСЫВАЕМЫЕ ТОПИКИ:
----------------------
/cmd_vel (geometry_msgs/Twist):
    - Команды скорости от Nav2 или других нод
    - linear.x: скорость вперед/назад (м/с)
    - angular.z: скорость поворота (рад/с)
    - Аналогия: "Газ и руль" для робота

ВАЖНО: odometry_node ТОЛЬКО считает одометрию, не управляет моторами!
Управление моторами выполняет chassis_node, который также слушает /cmd_vel
===================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster
import math
import time


class OdometryNode(Node):
    """
    Класс ноды одометрии.

    Наследуется от Node - базового класса для всех ROS2 нод.
    Аналогия из Java: похоже на extends Thread или extends HttpServlet
    """

    # ============================================================================
    # КОНСТАНТЫ РОБОТА ДЛЯ ЭНКОДЕРНОЙ ОДОМЕТРИИ
    # ============================================================================
    WHEEL_CIRCUMFERENCE = 0.59  # м - длина окружности колеса (откалибровано)
    GEAR_RATIO = 4.0007         # передаточное число редуктора
    ENCODER_RESOLUTION = 4096   # разрешение энкодера AS5600 (шагов на оборот)
    WHEEL_BASE = 0.363          # м - расстояние между колёсами (откалибровано)

    # Метров на один шаг энкодера
    METERS_PER_STEP = WHEEL_CIRCUMFERENCE / (ENCODER_RESOLUTION * GEAR_RATIO)

    def __init__(self):
        """
        Конструктор ноды. Инициализирует все необходимые компоненты.

        Порядок действий:
        1. Вызвать конструктор родителя (как super() в Java)
        2. Инициализировать переменные состояния
        3. Настроить публикаторы (publishers)
        4. Настроить подписчиков (subscribers)
        5. Запустить таймер для периодических обновлений
        """
        # Вызываем конструктор родительского класса
        # 'odometry_node' - имя ноды в системе ROS2
        super().__init__('odometry_node')

        # Параметр: публиковать ли TF (odom -> base_footprint)
        # Отключается когда EKF (robot_localization) берёт на себя TF
        self.declare_parameter('publish_tf', True)
        self.publish_tf = self.get_parameter('publish_tf').value

        # ============================================================================
        # ПЕРЕМЕННЫЕ СОСТОЯНИЯ РОБОТА
        # ============================================================================

        # Текущая позиция робота (в метрах и радианах)
        self.x = 0.0              # Позиция по оси X (вперед/назад)
        self.y = 0.0              # Позиция по оси Y (влево/вправо)
        self.theta = 0.0          # Угол поворота (ориентация робота в радианах)

        # Текущая скорость робота (в м/с и рад/с)
        self.vx = 0.0             # Линейная скорость по X
        self.vy = 0.0             # Линейная скорость по Y (для holonomic роботов, у нас = 0)
        self.vth = 0.0            # Угловая скорость (скорость поворота)

        # Временные метки для вычисления dt (delta time)
        # Аналогия: System.currentTimeMillis() в Java
        self.last_time = time.time()
        self.current_time = time.time()

        # ============================================================================
        # КОНФИГУРАЦИЯ РОБОТА
        # ============================================================================
        # ВАЖНО: Эти параметры нужно будет настроить под ваш конкретный робот!

        # Максимальные скорости (для безопасности и реалистичности)
        self.max_linear_velocity = 0.5    # Максимум 0.5 м/с (50 см/сек)
        self.max_angular_velocity = 1.0   # Максимум 1.0 рад/с (~57 градусов/сек)

        # Коэффициент масштабирования для команд (если нужна калибровка)
        # Если робот едет слишком быстро/медленно, можно подкрутить эти значения
        # Калибровка: робот проезжает в 1.35 раза больше чем показывает одометрия
        # (тест: одометрия 1.0м → реально 1.40м, одометрия 2.0м → реально 2.6м)
        # self.linear_scale = 1.35
        self.linear_scale = 1.0
        self.angular_scale = 2.0

        # ============================================================================
        # НАСТРОЙКА ПУБЛИКАТОРОВ
        # ============================================================================
        # Publishers отправляют данные в топики
        # Аналогия из Java: MessageProducer в JMS или Publisher в pub/sub системах

        # Публикатор одометрии
        # Отправляет полную информацию о позиции и скорости робота
        self.odom_publisher = self.create_publisher(
            Odometry,           # Тип сообщения (как класс DTO в Java)
            '/odom',            # Имя топика (как название очереди в JMS)
            10                  # Размер очереди (если сообщения идут быстрее, чем обрабатываются)
        )
        self.get_logger().info('Публикатор одометрии создан для топика /odom')

        # ============================================================================
        # TF BROADCASTER
        # ============================================================================
        # TransformBroadcaster публикует трансформации между системами координат
        # Это КРИТИЧЕСКИ ВАЖНО для Nav2!
        #
        # Объяснение: В ROS2 каждая часть робота имеет свою систему координат.
        # TF broadcaster говорит "где base_link относительно odom"
        # Nav2 использует это чтобы понять где робот на карте
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('TF broadcaster инициализирован')

        # ============================================================================
        # НАСТРОЙКА ПОДПИСЧИКОВ
        # ============================================================================
        # Subscribers получают данные из топиков
        # Аналогия из Java: MessageConsumer в JMS или Subscriber в pub/sub

        # Подписчик на команды скорости (стандартный интерфейс Nav2)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,                      # Тип сообщения
            '/cmd_vel',                 # Топик со стандартными командами скорости
            self.cmd_vel_callback,      # Функция-обработчик (callback)
            10                          # Размер очереди
        )
        self.get_logger().info('Подписчик создан для топика /cmd_vel')

        # ============================================================================
        # ТАЙМЕР ДЛЯ ОБНОВЛЕНИЯ ОДОМЕТРИИ
        # ============================================================================
        # Таймер вызывает функцию с определенной частотой
        # Аналогия: ScheduledExecutorService.scheduleAtFixedRate() в Java

        # Частота обновления: 50 Гц (50 раз в секунду = каждые 0.02 секунды)
        # Это стандартная частота для одометрии в ROS2
        update_frequency = 50.0  # Гц
        timer_period = 1.0 / update_frequency  # Период в секундах

        self.timer = self.create_timer(
            timer_period,                   # Период вызова
            self.update_odometry_callback   # Функция которая будет вызываться
        )
        self.get_logger().info(f'Таймер одометрии запущен с частотой {update_frequency} Гц')

        # Флаг для отслеживания активности движения
        self.is_moving = False

        # ============================================================================
        # СОСТОЯНИЕ ЭНКОДЕРНОЙ ОДОМЕТРИИ
        # ============================================================================
        self.last_left_steps = None
        self.last_right_steps = None
        self.encoder_initialized = False
        self.odometry_source = 'cmd_vel'  # 'cmd_vel' или 'encoder'
        self.last_encoder_time = time.time()
        self.last_encoder_callback_time = None  # Для расчёта скорости из энкодеров
        self.encoder_timeout = 1.0  # секунды - таймаут для fallback на cmd_vel

        # ============================================================================
        # ПОДПИСКА НА ДАННЫЕ ЭНКОДЕРОВ
        # ============================================================================
        self.encoder_subscriber = self.create_subscription(
            Int64MultiArray, '/wheel_encoders',
            self.encoder_callback, 10
        )
        self.get_logger().info('Подписчик создан для топика /wheel_encoders')

        self.get_logger().info('='*80)
        self.get_logger().info('Нода одометрии успешно инициализирована!')
        self.get_logger().info('Начальная позиция: x=0.0, y=0.0, theta=0.0')
        self.get_logger().info(f'Ожидание данных энкодеров (fallback на cmd_vel)')
        self.get_logger().info('='*80)


    def cmd_vel_callback(self, msg):
        """
        Callback-функция для обработки команд скорости из топика /cmd_vel.

        Эта функция вызывается автоматически каждый раз, когда приходит новое
        сообщение в топик /cmd_vel (например, от Nav2).

        ВАЖНО: Скорость обновляется только если используется cmd_vel одометрия.
        При энкодерной одометрии скорость рассчитывается из данных энкодеров.

        Аналогия из Java: Метод onMessage() в JMS MessageListener

        Args:
            msg (Twist): Сообщение с командами скорости
                msg.linear.x  - скорость вперед/назад (м/с)
                msg.linear.y  - скорость влево/вправо (обычно 0 для дифф. привода)
                msg.angular.z - скорость поворота (рад/с)
        """
        # Обновляем скорость только если используем cmd_vel одометрию (fallback)
        # При энкодерной одометрии скорость рассчитывается в encoder_callback
        if self.odometry_source != 'encoder':
            # Извлекаем линейную скорость (вперед/назад)
            # Ограничиваем максимальной скоростью для безопасности
            linear_x = max(min(msg.linear.x, self.max_linear_velocity),
                           -self.max_linear_velocity)

            # Извлекаем угловую скорость (поворот)
            # Ограничиваем максимальной угловой скоростью
            angular_z = max(min(msg.angular.z, self.max_angular_velocity),
                            -self.max_angular_velocity)

            # Применяем коэффициенты масштабирования (для калибровки)
            self.vx = linear_x * self.linear_scale
            self.vth = angular_z * self.angular_scale

            # Для differential drive (дифференциальный привод) робота
            # боковая скорость всегда 0
            self.vy = 0.0

        # Логируем полученные команды (для отладки)
        cmd_vx = msg.linear.x
        cmd_vth = msg.angular.z
        if abs(cmd_vx) > 0.01 or abs(cmd_vth) > 0.01:
            if not self.is_moving:
                self.get_logger().info(f'Начало движения: cmd_vx={cmd_vx:.3f} м/с, cmd_vth={cmd_vth:.3f} рад/с')
                self.is_moving = True
        else:
            if self.is_moving:
                self.get_logger().info('Остановка')
                self.is_moving = False


    def encoder_callback(self, msg):
        """
        Callback-функция для обработки данных энкодеров из топика /wheel_encoders.

        Эта функция вызывается каждый раз, когда приходят данные с энкодеров.
        Расчёт одометрии (позиция И скорость) на основе реальных данных с колёс (closed-loop).

        Args:
            msg (Int64MultiArray): Сообщение с данными энкодеров
                msg.data[0] - шаги левого колеса (totalSteps)
                msg.data[1] - шаги правого колеса (totalSteps)
                msg.data[2] - временная метка Arduino (millis)
        """
        if len(msg.data) < 3:
            return

        left_steps = msg.data[0]
        right_steps = msg.data[1]
        # timestamp_ms = msg.data[2]  # Можно использовать для синхронизации

        # Получаем текущее время для расчёта скорости
        current_callback_time = time.time()

        # Обновляем время последнего получения данных энкодеров
        self.last_encoder_time = current_callback_time

        # Инициализация при первом получении данных
        if not self.encoder_initialized:
            self.last_left_steps = left_steps
            self.last_right_steps = right_steps
            self.last_encoder_callback_time = current_callback_time
            self.encoder_initialized = True
            self.odometry_source = 'encoder'
            self.get_logger().info('Переключение на энкодерную одометрию (closed-loop)')
            return

        # Переключение обратно на encoder если были на cmd_vel (после таймаута)
        if self.odometry_source != 'encoder':
            self.odometry_source = 'encoder'
            self.get_logger().info('Восстановление энкодерной одометрии (closed-loop)')

        # Вычисляем дельты шагов
        delta_left = left_steps - self.last_left_steps
        delta_right = right_steps - self.last_right_steps

        # Вычисляем dt для расчёта скорости
        dt = current_callback_time - self.last_encoder_callback_time

        # Защита от деления на ноль и слишком большого dt
        if dt <= 0 or dt > 1.0:
            self.last_left_steps = left_steps
            self.last_right_steps = right_steps
            self.last_encoder_callback_time = current_callback_time
            return

        # Если нет изменений - робот стоит, обнуляем скорость
        if delta_left == 0 and delta_right == 0:
            self.vx = 0.0
            self.vth = 0.0
            self.vy = 0.0
            self.last_encoder_callback_time = current_callback_time
            return

        # Переводим шаги в метры
        dist_left = delta_left * self.METERS_PER_STEP
        dist_right = delta_right * self.METERS_PER_STEP

        # Дифференциальный привод: формулы для расчёта движения
        delta_distance = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.WHEEL_BASE

        # ============================================================================
        # РАСЧЁТ АКТУАЛЬНОЙ СКОРОСТИ ИЗ ЭНКОДЕРОВ
        # ============================================================================
        # Линейная скорость = пройденное расстояние / время
        self.vx = delta_distance / dt

        # Угловая скорость = изменение угла / время
        self.vth = delta_theta / dt

        # Боковая скорость всегда 0 для дифференциального привода
        self.vy = 0.0

        # ============================================================================
        # ОБНОВЛЕНИЕ ПОЗИЦИИ
        # ============================================================================
        # Используем средний угол для более точного расчёта
        avg_theta = self.theta + delta_theta / 2.0
        self.x += delta_distance * math.cos(avg_theta)
        self.y += delta_distance * math.sin(avg_theta)
        self.theta += delta_theta

        # Нормализуем угол в диапазон [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Сохраняем текущие значения для следующей итерации
        self.last_left_steps = left_steps
        self.last_right_steps = right_steps
        self.last_encoder_callback_time = current_callback_time


    def update_odometry_callback(self):
        """
        Callback-функция таймера для обновления одометрии.

        Вызывается 50 раз в секунду (каждые 0.02 сек).

        ЧТО ЗДЕСЬ ПРОИСХОДИТ:
        1. Проверяем источник одометрии (encoder или cmd_vel)
        2. Если encoder - позиция уже обновлена в encoder_callback
        3. Если cmd_vel - вычисляем позицию из команд скорости (fallback)
        4. Публикуем одометрию в топик /odom
        5. Публикуем TF трансформацию odom -> base_link

        Аналогия: Как run() метод в Thread или метод в @Scheduled аннотации Spring
        """
        # ============================================================================
        # ШАГ 0: ПРОВЕРКА ИСТОЧНИКА ОДОМЕТРИИ
        # ============================================================================
        # Проверяем таймаут энкодеров для fallback на cmd_vel
        if self.odometry_source == 'encoder':
            if time.time() - self.last_encoder_time > self.encoder_timeout:
                self.odometry_source = 'cmd_vel'
                self.get_logger().warn('Таймаут данных энкодеров - переключение на cmd_vel (open-loop)')

        # ============================================================================
        # ШАГ 1: ВЫЧИСЛЕНИЕ ВРЕМЕННОГО ИНТЕРВАЛА (dt)
        # ============================================================================
        # Получаем текущее время
        self.current_time = time.time()

        # Вычисляем сколько секунд прошло с последнего обновления
        dt = self.current_time - self.last_time

        # Если прошло слишком много времени (например, нода была на паузе),
        # ограничиваем dt чтобы не было скачков
        if dt > 1.0:  # Больше 1 секунды - что-то не так
            self.get_logger().warn(f'Большой временной интервал: {dt:.3f} сек. Сбрасываем.')
            dt = 0.02  # Устанавливаем стандартное значение

        # ============================================================================
        # ШАГ 2: ВЫЧИСЛЕНИЕ ИЗМЕНЕНИЯ ПОЗИЦИИ (только для cmd_vel fallback)
        # ============================================================================
        # Если используется энкодерная одометрия, позиция уже обновлена в encoder_callback
        if self.odometry_source == 'cmd_vel':
            # Формулы одометрии для дифференциального привода:
            #
            # Если робот поворачивает (vth != 0):
            #   - Робот движется по дуге окружности
            #   - Используем формулы для движения по окружности
            #
            # Если робот не поворачивает (vth == 0):
            #   - Робот движется прямо
            #   - Простая линейная интерполяция

            # Вычисляем изменение угла (delta_theta)
            delta_theta = self.vth * dt

            # Вычисляем изменение позиции (delta_x, delta_y)
            # Используем текущий угол робота для перевода в глобальные координаты
            if abs(self.vth) < 1e-6:  # Практически нет поворота
                # Прямолинейное движение
                delta_x = self.vx * dt * math.cos(self.theta)
                delta_y = self.vx * dt * math.sin(self.theta)
            else:
                # Движение по дуге
                # Радиус поворота
                radius = self.vx / self.vth

                # Изменение позиции при движении по дуге
                delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
                delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

            # ============================================================================
            # ШАГ 3: ОБНОВЛЕНИЕ СОСТОЯНИЯ РОБОТА (только для cmd_vel)
            # ============================================================================
            # Обновляем позицию
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Нормализуем угол в диапазон [-π, π]
            # Аналогия: как нормализация угла 370° -> 10°
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Сохраняем текущее время для следующей итерации
        self.last_time = self.current_time

        # ============================================================================
        # ШАГ 4: ПУБЛИКАЦИЯ ОДОМЕТРИИ
        # ============================================================================
        self._publish_odometry()

        # ============================================================================
        # ШАГ 5: ПУБЛИКАЦИЯ TF ТРАНСФОРМАЦИИ (если не отключена)
        # ============================================================================
        if self.publish_tf:
            self._publish_transform()


    def _publish_odometry(self):
        """
        Публикует сообщение одометрии в топик /odom.

        Сообщение Odometry содержит:
        - header: временная метка и система координат
        - pose: позиция и ориентация робота
        - twist: линейные и угловые скорости
        """
        # Создаем сообщение одометрии
        odom_msg = Odometry()

        # ============================================================================
        # ЗАПОЛНЕНИЕ HEADER (метаданные сообщения)
        # ============================================================================
        # stamp - временная метка (когда было создано сообщение)
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        # frame_id - система координат в которой задана позиция
        # 'odom' - это фиксированная система координат, где робот стартовал
        odom_msg.header.frame_id = 'odom'

        # child_frame_id - система координат робота (его тело)
        odom_msg.child_frame_id = 'base_footprint'

        # ============================================================================
        # ЗАПОЛНЕНИЕ POSE (позиция и ориентация)
        # ============================================================================
        # Позиция в 3D пространстве (x, y, z)
        # z = 0 потому что робот на плоскости
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Ориентация (куда смотрит робот) в формате кватерниона
        # Кватернион - это математический способ представления вращения в 3D
        # Мы конвертируем угол theta (в радианах) в кватернион
        quaternion = self._euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = quaternion

        # Ковариационная матрица позиции (uncertainty)
        # Показывает насколько мы уверены в измерениях
        # Формат: 6x6 матрица для [x, y, z, roll, pitch, yaw]
        # Диагональные элементы - дисперсия по каждой оси
        # Чем больше значение - тем меньше уверенность
        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,   # x
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,   # y
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,   # z (не используется)
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,   # roll (не используется)
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,   # pitch (не используется)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1    # yaw (theta)
        ]

        # ============================================================================
        # ЗАПОЛНЕНИЕ TWIST (скорости)
        # ============================================================================
        # Линейные скорости
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0

        # Угловые скорости
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.vth

        # Ковариационная матрица скоростей
        odom_msg.twist.covariance = [
            0.05, 0.0,  0.0, 0.0, 0.0, 0.0,   # vx
            0.0,  0.05, 0.0, 0.0, 0.0, 0.0,   # vy
            0.0,  0.0,  0.1, 0.0, 0.0, 0.0,   # vz
            0.0,  0.0,  0.0, 0.1, 0.0, 0.0,   # angular x
            0.0,  0.0,  0.0, 0.0, 0.1, 0.0,   # angular y
            0.0,  0.0,  0.0, 0.0, 0.0, 0.05   # angular z
        ]

        # Публикуем сообщение
        self.odom_publisher.publish(odom_msg)

        # Периодически логируем позицию (каждую секунду)
        if hasattr(self, '_last_log_time'):
            if self.current_time - self._last_log_time > 1.0:
                source_label = '[ENC]' if self.odometry_source == 'encoder' else '[CMD]'
                self.get_logger().info(
                    f'{source_label} Позиция: x={self.x:.3f}м, y={self.y:.3f}м, θ={math.degrees(self.theta):.1f}° | '
                    f'Скорость: {self.vx:.2f}м/с, {math.degrees(self.vth):.1f}°/с'
                )
                self._last_log_time = self.current_time
        else:
            self._last_log_time = self.current_time


    def _publish_transform(self):
        """
        Публикует TF трансформацию между системами координат odom и base_link.

        TF (Transform) - это дерево координатных систем в ROS2.
        Каждая часть робота имеет свою систему координат, и TF показывает
        как они связаны между собой.

        Для Nav2 КРИТИЧЕСКИ ВАЖНО чтобы была опубликована трансформация
        odom -> base_link, иначе Nav2 не будет работать!
        """
        # Создаем сообщение трансформации
        transform = TransformStamped()

        # Временная метка
        transform.header.stamp = self.get_clock().now().to_msg()

        # Родительская система координат (откуда)
        transform.header.frame_id = 'odom'

        # Дочерняя система координат (куда)
        transform.child_frame_id = 'base_footprint'

        # Позиция base_footprint в системе координат odom
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # Ориентация base_link в системе координат odom
        quaternion = self._euler_to_quaternion(0, 0, self.theta)
        transform.transform.rotation = quaternion

        # Отправляем трансформацию
        self.tf_broadcaster.sendTransform(transform)


    def _euler_to_quaternion(self, roll, pitch, yaw):
        """
        Конвертирует углы Эйлера (roll, pitch, yaw) в кватернион.

        ЧТО ТАКОЕ УГЛЫ ЭЙЛЕРА?
        - Roll: крен (наклон влево/вправо)
        - Pitch: тангаж (нос вверх/вниз)
        - Yaw: рысканье (поворот влево/вправо) - это наш theta!

        ЧТО ТАКОЕ КВАТЕРНИОН?
        Математический способ представления ориентации в 3D пространстве.
        Используется в ROS2 потому что не имеет проблемы "gimbal lock"
        и удобен для интерполяции.

        Формат: (x, y, z, w) - четыре числа

        Args:
            roll (float): Крен в радианах
            pitch (float): Тангаж в радианах
            yaw (float): Рысканье в радианах (наш theta)

        Returns:
            Quaternion: Ориентация в формате кватерниона
        """
        # Формулы конвертации Euler -> Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion = Quaternion()
        quaternion.w = cr * cp * cy + sr * sp * sy
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy

        return quaternion


    def destroy_node(self):
        """
        Корректное завершение работы ноды.

        Вызывается при остановке ноды (Ctrl+C или shutdown).
        Аналогия из Java: метод close() в AutoCloseable или destroy() в Spring
        """
        self.get_logger().info('Завершение работы ноды одометрии')
        self.get_logger().info(f'Финальная позиция: x={self.x:.3f}м, y={self.y:.3f}м, θ={math.degrees(self.theta):.1f}°')

        # Вызываем родительский метод для корректного завершения
        super().destroy_node()


def main(args=None):
    """
    Точка входа в программу.

    Эта функция:
    1. Инициализирует ROS2
    2. Создает ноду
    3. Запускает цикл обработки (spin)
    4. Корректно завершает работу при остановке

    Аналогия из Java: метод main() в Spring Boot Application
    """
    # Инициализируем ROS2
    # Это ОБЯЗАТЕЛЬНО нужно сделать перед созданием любой ноды
    rclpy.init(args=args)

    # Создаем экземпляр ноды одометрии
    odometry_node = OdometryNode()

    try:
        # Запускаем цикл обработки
        # spin() блокирует выполнение и обрабатывает все callbacks
        # (таймеры, подписчики и т.д.)
        # Аналогия: бесконечный while(true) с обработкой событий
        rclpy.spin(odometry_node)

    except KeyboardInterrupt:
        # Пользователь нажал Ctrl+C
        odometry_node.get_logger().info('Прерывание пользователем (Ctrl+C)')

    finally:
        # Корректно завершаем работу в любом случае
        odometry_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Если файл запущен напрямую (не импортирован как модуль)
    main()
