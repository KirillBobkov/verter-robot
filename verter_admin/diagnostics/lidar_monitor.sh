#!/bin/bash
#
# Мониторинг и диагностика лидара RPLiDAR A1M8
# Выводит данные лидара в консоль непрерывно
#

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  МОНИТОРИНГ ЛИДАРА RPLiDAR     ${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# ============================================================================
# Проверка подключения
# ============================================================================
echo -e "${BLUE}=== Проверка подключения ===${NC}"
echo ""

# Проверяем USB устройства
if lsusb | grep -q "10c4:ea60"; then
    echo -e "${GREEN}✓ CP2102 (10c4:ea60) найден${NC}"
elif lsusb | grep -q "1a86:7523"; then
    echo -e "${GREEN}✓ CH340 (1a86:7523) найден${NC}"
else
    echo -e "${RED}✗ USB-конвертер лидара не найден${NC}"
    echo "Проверьте подключение лидара к USB хабу"
    exit 1
fi

# Проверяем порт
if [ -L /dev/rplidar ]; then
    LIDAR_PORT="/dev/rplidar"
    echo -e "${GREEN}✓ Symlink /dev/rplidar существует${NC}"
    echo "  Цель: $(readlink /dev/rplidar)"
elif [ -e /dev/ttyUSB0 ]; then
    LIDAR_PORT="/dev/ttyUSB0"
    echo -e "${YELLOW}⚠ /dev/rplidar не найден, используем /dev/ttyUSB0${NC}"
else
    echo -e "${RED}✗ Serial порт не найден${NC}"
    exit 1
fi

# Проверяем права
if [ -r "$LIDAR_PORT" ] && [ -w "$LIDAR_PORT" ]; then
    echo -e "${GREEN}✓ Права доступа в порядке${NC}"
else
    echo -e "${RED}✗ Нет прав доступа к $LIDAR_PORT${NC}"
    exit 1
fi

# Проверяем, используется ли порт
if lsof "$LIDAR_PORT" >/dev/null 2>&1; then
    echo -e "${YELLOW}⚠ Порт используется другим процессом:${NC}"
    lsof "$LIDAR_PORT" | tail -1
    echo ""
    echo "Остановите процесс или используйте другой порт"
    exit 1
fi

echo ""
echo -e "${GREEN}Порт: $LIDAR_PORT${NC}"
echo -e "${GREEN}Скорость: 115200 baud${NC}"
echo ""

# ============================================================================
# Мониторинг данных
# ============================================================================
echo -e "${BLUE}=== Мониторинг данных ===${NC}"
echo ""
echo -e "${CYAN}Чтение данных лидара (Ctrl+C для остановки)...${NC}"
echo ""

# Настройка порта
stty -F "$LIDAR_PORT" 115200 cs8 -cstopb -parenb -ixon -ixoff 2>/dev/null || true

# Счетчик
COUNT=0
HEADER_COUNT=0
SCAN_DATA_COUNT=0

# Функция для парсинга данных RPLiDAR
parse_lidar_data() {
    local hex_data="$1"
    
    # Проверяем заголовок пакета
    if echo "$hex_data" | grep -q "a5 5a"; then
        echo -e "${CYAN}  ┌─ ПАКЕТ RPLiDAR${NC}"
        
        # Извлекаем байты данных (без адреса xxd)
        local bytes=$(echo "$hex_data" | sed 's/^[0-9a-f]*: //' | tr -d ' \n')
        
        # Проверяем длину данных (минимум 10 байт для заголовка + типа)
        if [ ${#bytes} -ge 20 ]; then
            # Байт 2: длина пакета (hex)
            local len_hex="${bytes:4:2}"
            local len_dec=$((16#$len_hex))
            
            # Байт 3: режим
            local mode_hex="${bytes:6:2}"
            
            # Байт 4: тип пакета
            local type_hex="${bytes:8:2}"
            
            case "$type_hex" in
                81)
                    echo -e "${CYAN}  │ Тип: СКАН ДАННЫХ (0x81)${NC}"
                    echo -e "${CYAN}  │ Длина: $len_dec байт${NC}"
                    echo -e "${CYAN}  │ Режим: 0x$mode_hex${NC}"
                    
                    # Парсим измерения (начиная с байта 5)
                    local offset=10
                    local points=0
                    
                    while [ $((offset + 8)) -le ${#bytes} ] && [ $points -lt 4 ]; do
                        # Quality (1 байт)
                        local quality_hex="${bytes:$offset:2}"
                        local quality=$((16#$quality_hex))
                        
                        # Angle (2 байта, little endian)
                        local angle_low="${bytes:$((offset+2)):2}"
                        local angle_high="${bytes:$((offset+4)):2}"
                        local angle_raw=$((16#$angle_low + 16#$angle_high * 256))
                        local angle_deg=$(echo "scale=2; $angle_raw / 64.0" | bc)
                        
                        # Distance (2 байта, little endian)
                        local dist_low="${bytes:$((offset+6)):2}"
                        local dist_high="${bytes:$((offset+8)):2}"
                        local distance_mm=$((16#$dist_low + 16#$dist_high * 256))
                        local distance_cm=$(echo "scale=1; $distance_mm / 10.0" | bc)
                        
                        if [ $distance_mm -gt 0 ]; then
                            echo -e "${CYAN}  │   Точка $((points+1)): Q=$quality, Угол=${angle_deg}°, Дист=${distance_cm}см${NC}"
                        fi
                        
                        offset=$((offset + 10))
                        points=$((points + 1))
                    done
                    ;;
                06)
                    echo -e "${CYAN}  │ Тип: ИНФОРМАЦИЯ УСТРОЙСТВА (0x06)${NC}"
                    echo -e "${CYAN}  │ Длина: $len_dec байт${NC}"
                    ;;
                15)
                    echo -e "${CYAN}  │ Тип: ЗДОРОВЬЕ (0x15)${NC}"
                    local status_hex="${bytes:10:2}"
                    local status=$((16#$status_hex))
                    if [ $status -eq 0 ]; then
                        echo -e "${GREEN}  │ Статус: ХОРОШО${NC}"
                    else
                        echo -e "${RED}  │ Статус: ОШИБКА (0x$status_hex)${NC}"
                    fi
                    ;;
                04)
                    echo -e "${CYAN}  │ Тип: СТАРТ СКАНА (0x04)${NC}"
                    ;;
                *)
                    echo -e "${YELLOW}  │ Тип: НЕИЗВЕСТНЫЙ (0x$type_hex)${NC}"
                    ;;
            esac
        fi
        echo -e "${CYAN}  └──────────────────${NC}"
        SCAN_DATA_COUNT=$((SCAN_DATA_COUNT + 1))
    fi
}

echo -e "${YELLOW}Легенда:${NC}"
echo -e "${GREEN}  ✓  Успешно${NC}"
echo -e "${YELLOW}  ⚠  Предупреждение${NC}"
echo -e "${RED}  ✗  Ошибка${NC}"
echo -e "${CYAN}  ── Данные лидара${NC}"
echo ""

while true; do
    # Читаем данные в течение 0.5 секунды
    DATA=$(timeout 0.5 cat "$LIDAR_PORT" 2>/dev/null | xxd -c 16 || true)
    
    if [ -n "$DATA" ]; then
        COUNT=$((COUNT + 1))
        
        # Парсим и выводим данные в читаемом формате
        parse_lidar_data "$DATA"
        
        # Показываем сырые данные в сжатом виде для отладки
        local raw_preview=$(echo "$DATA" | head -1 | sed 's/^[0-9a-f]*: //' | cut -c1-60)
        echo -e "${GREEN}[$COUNT]${NC} ${YELLOW}Raw:${NC} $raw_preview..."
    else
        echo -e "${RED}[$COUNT] Нет данных${NC}"
        echo -e "${RED}  Убедитесь, что лидар подключен к питанию и вращается${NC}"
    fi
    
    # Каждые 50 пакетов выводим статистику
    if [ $((COUNT % 50)) -eq 0 ]; then
        echo ""
        echo -e "${BLUE}═══════════════════════════════════════${NC}"
        echo -e "${BLUE}            СТАТИСТИКА${NC}"
        echo -e "${BLUE}═══════════════════════════════════════${NC}"
        echo -e "${GREEN}Всего пакетов:${NC}      $COUNT"
        echo -e "${GREEN}Пакетов RPLiDAR:${NC}   $SCAN_DATA_COUNT"
        echo -e "${GREEN}Заголовков:${NC}        $HEADER_COUNT"
        if [ $COUNT -gt 0 ]; then
            local rate=$(echo "scale=1; $SCAN_DATA_COUNT * 100 / $COUNT" | bc)
            echo -e "${GREEN}Качество данных:${NC}    ${rate}%"
        fi
        echo -e "${BLUE}═══════════════════════════════════════${NC}"
        echo ""
    fi
    
    sleep 0.5
done
