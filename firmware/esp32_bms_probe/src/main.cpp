#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
// Standalone diagnostic only: no ROS, BLE, motor commands, BMS writes or periodic polls.
static constexpr uart_port_t PORT = UART_NUM_2;
static uint32_t bytesReceived = 0, lastReport = 0;
static String command;
static bool uartReady = false;
void readOnce() {
    // JK legacy 4E57 READ_ALL (0x06), computer source (0x03).
    const uint8_t request[] = {0x4e, 0x57, 0, 0x13, 0, 0, 0, 0, 6, 3, 0, 0, 0, 0, 0, 0, 0x68, 0, 0, 1, 0x29};
    if (!uartReady)
        return;
    uart_set_pin(PORT, BMS_TX, BMS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_write_bytes(PORT, reinterpret_cast<const char *>(request), sizeof(request));
    esp_err_t sent = uart_wait_tx_done(PORT, pdMS_TO_TICKS(1000));
    gpio_reset_pin(static_cast<gpio_num_t>(BMS_TX));
    pinMode(BMS_TX, INPUT); // disconnect transmitter again
    if (sent != ESP_OK) {
        Serial.println("ERROR TX_TIMEOUT");
        return;
    }
    Serial.println("TX READ_ALL 4E5700130000000006030000000000006800000129");
}
void setup() {
    Serial.begin(115200);
    pinMode(BMS_TX, INPUT);
    uart_config_t cfg = {};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;
    esp_err_t a = uart_param_config(PORT, &cfg);
    esp_err_t b = uart_set_pin(PORT, UART_PIN_NO_CHANGE, BMS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    esp_err_t c = uart_driver_install(PORT, 4096, 0, 0, nullptr, 0);
    uartReady = (a == ESP_OK && b == ESP_OK && c == ESP_OK);
    Serial.printf("BOOT ready=%d RX=%d TX=%d baud=115200 passive=1\n", uartReady, BMS_RX, BMS_TX);
    Serial.println("Commands: READ (one read-only request), BAUD 9600, BAUD 115200, STATUS. No automatic transmit.");
}
void loop() {
    if (uartReady) {
        uint8_t buf[128];
        int n = uart_read_bytes(PORT, buf, sizeof(buf), 0);
        if (n > 0) {
            bytesReceived += n;
            Serial.printf("RX %lu ", static_cast<unsigned long>(millis()));
            for (int i = 0; i < n; i++)
                Serial.printf("%02X", buf[i]);
            Serial.println();
        }
    }
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            command.trim();
            if (command == "READ")
                readOnce();
            else if (command.startsWith("BAUD ")) {
                const uint32_t rates[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
                uint32_t baud = command.substring(5).toInt();
                bool allowed = false;
                for (auto rate : rates)
                    if (command == String("BAUD ") + String(rate))
                        allowed = true;
                if (allowed && uartReady && uart_set_baudrate(PORT, baud) == ESP_OK)
                    Serial.println(command);
                else
                    Serial.println("ERROR baud rejected");
            } else if (command == "STATUS")
                Serial.printf("STATUS rx_bytes=%lu ready=%d\n", static_cast<unsigned long>(bytesReceived), uartReady);
            else if (command.length())
                Serial.println("ERROR unknown command");
            command = "";
        } else if (command.length() < 40)
            command += c;
        else
            command = "";
    }
    if (millis() - lastReport >= 5000) {
        lastReport = millis();
        Serial.printf("STATUS rx_bytes=%lu ready=%d\n", static_cast<unsigned long>(bytesReceived), uartReady);
    }
    delay(1);
}
