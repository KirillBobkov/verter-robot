#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"

// Receive-only diagnostics: neither GPIO16 nor GPIO17 is ever a UART TX.
static const uart_port_t U = UART_NUM_2;
static QueueHandle_t events;
static bool ready = false, active = false;
static int rx = 16, baud = 115200;
static String cmd;
static uint32_t count, zeros, frames, parity, breaks, fifo, full;
static uint32_t low, high, legacy, modern, shift, started, reported;
static uint8_t sample[256];
static size_t sampled;

void report() {
    Serial.printf(
        "STAT {\"rx\":%d,\"baud\":%d,\"ms\":%lu,\"bytes\":%lu,\"zeros\":%lu,\"frame_errors\":%lu,\"parity_errors\":%lu,\"breaks\":%lu,\"fifo_overflows\":%lu,\"buffer_full\":%lu,\"pin_low_samples\":%lu,\"pin_high_samples\":%lu,\"prefix_4e57\":%lu,\"prefix_55aaeb90\":%lu,\"sample\":\"",
        rx, baud, (unsigned long)(millis() - started), (unsigned long)count, (unsigned long)zeros,
        (unsigned long)frames, (unsigned long)parity, (unsigned long)breaks, (unsigned long)fifo, (unsigned long)full,
        (unsigned long)low, (unsigned long)high, (unsigned long)legacy, (unsigned long)modern);
    for (size_t i = 0; i < sampled; i++)
        Serial.printf("%02X", sample[i]);
    Serial.println("\"}");
}
void beginPhase(int pin, int rate) {
    active = false;
    gpio_reset_pin(GPIO_NUM_16);
    gpio_reset_pin(GPIO_NUM_17);
    pinMode(16, INPUT);
    pinMode(17, INPUT);
    if (uart_set_pin(U, UART_PIN_NO_CHANGE, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
        uart_set_baudrate(U, rate) != ESP_OK) {
        Serial.println("ERROR configure");
        return;
    }
    uart_flush_input(U);
    xQueueReset(events);
    rx = pin;
    baud = rate;
    count = zeros = frames = parity = breaks = fifo = full = low = high = legacy = modern = shift = 0;
    sampled = 0;
    started = reported = millis();
    active = true;
    Serial.printf("BEGIN %d %d\n", rx, baud);
}
void setup() {
    Serial.begin(115200);
    pinMode(16, INPUT);
    pinMode(17, INPUT);
    uart_config_t cfg = {};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;
    ready = uart_param_config(U, &cfg) == ESP_OK &&
            uart_set_pin(U, UART_PIN_NO_CHANGE, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK &&
            uart_driver_install(U, 16384, 0, 256, &events, 0) == ESP_OK;
    Serial.printf("PASSIVE_DIAG ready=%d no_tx=1\n", ready);
}
void loop() {
    if (ready) {
        uart_event_t e;
        // Bounded draining keeps the USB command interface responsive.
        for (int i = 0; i < 256 && xQueueReceive(events, &e, 0) == pdTRUE; i++) {
            if (!active)
                continue;
            switch (e.type) {
            case UART_FRAME_ERR:
                frames++;
                break;
            case UART_PARITY_ERR:
                parity++;
                break;
            case UART_BREAK:
                breaks++;
                break;
            case UART_FIFO_OVF:
                fifo++;
                break;
            case UART_BUFFER_FULL:
                full++;
                break;
            default:
                break;
            }
        }
        uint8_t buf[1024];
        int n = uart_read_bytes(U, buf, sizeof(buf), 0);
        if (active) {
            if (digitalRead(rx))
                high++;
            else
                low++;
            for (int i = 0; i < n; i++) {
                count++;
                if (buf[i] == 0)
                    zeros++;
                if (sampled < sizeof(sample))
                    sample[sampled++] = buf[i];
                shift = (shift << 8) | buf[i];
                if ((shift & 0xffff) == 0x4e57)
                    legacy++;
                if (shift == 0x55aaeb90)
                    modern++;
            }
        }
    }
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n') {
            cmd.trim();
            int pin = 0, rate = 0;
            char extra;
            if (cmd == "STATUS") {
                Serial.printf("PASSIVE_DIAG ready=%d no_tx=1\n", ready);
                if (active)
                    report();
            } else if (cmd == "END") {
                if (active)
                    report();
                active = false;
                Serial.println("END");
            } else if (sscanf(cmd.c_str(), "BEGIN %d %d %c", &pin, &rate, &extra) == 2 && (pin == 16 || pin == 17)) {
                const int rates[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
                bool allowed = false;
                for (int r : rates)
                    if (r == rate)
                        allowed = true;
                if (allowed && ready)
                    beginPhase(pin, rate);
                else
                    Serial.println("ERROR rate");
            } else if (cmd.length())
                Serial.println("ERROR command");
            cmd = "";
        } else if (cmd.length() < 40)
            cmd += ch;
        else
            cmd = "";
    }
    if (active && millis() - reported >= 5000) {
        report();
        reported = millis();
    }
    delay(1);
}
