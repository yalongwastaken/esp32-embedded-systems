// Author: Anthony Yalong
// Description: Blinking LED program for the ESP32 developed using the ESP-IDF framework. 
// Includes RTOS multi-tasking and communication via UART for real-time LED control.

// Includes
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/FreeRTOS.h>

// Hardware configuration
#define LED_PIN GPIO_NUM_2
#define UART_NUM UART_NUM_0

// Task configuration
#define LED_STACK_SIZE 2048
#define LED_PRIORITY 5
#define CMD_HNDLR_STACK_SIZE 4096
#define CMD_HNDLR_PRIORITY 3

// UART configuration
#define UART_BAUD_RATE 115200
#define UART_DATA_BITS UART_DATA_8_BITS
#define UART_PARITY UART_PARITY_DISABLE
#define UART_STOP_BITS UART_STOP_BITS_1
#define UART_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE
#define UART_SOURCE_CLK UART_SCLK_DEFAULT
#define BUF_SIZE (1024)

// Command queue size
#define CMD_QUEUE_SIZE 10

// Static variables
static QueueHandle_t cmd_queue;
static SemaphoreHandle_t led_mutex;
static const char *TAG = "Blinking LED 2 (ESP-IDF)";

// LED control commands
typedef enum {
    LED_CMD_ON,
    LED_CMD_OFF,
    LED_CMD_SLOW,
    LED_CMD_FAST
} led_command_t;

// LED state structure
typedef struct {
    bool is_blinking;
    bool cur_level;
    TickType_t delay;
    led_command_t cur_command;
} led_state_t;

// Persistent LED state
static led_state_t led_state = {
    .is_blinking = false,
    .cur_level = false,
    .delay = pdMS_TO_TICKS(1000),
    .cur_command = LED_CMD_OFF,
};

// UART initialization function
esp_err_t init_uart(void) {
    // UART config
    const uart_config_t uart_config = {
       .baud_rate = UART_BAUD_RATE,
       .data_bits = UART_DATA_BITS,
       .parity = UART_PARITY,
       .stop_bits = UART_STOP_BITS,
       .flow_ctrl = UART_FLOW_CTRL,
       .source_clk = UART_SOURCE_CLK,
    };

    // UART install driver
    esp_err_t uart_driver_install_status = uart_driver_install(UART_NUM, 2 * BUF_SIZE, 0, 0, NULL, 0);
    if (uart_driver_install_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(uart_driver_install_status));
        return uart_driver_install_status;
    }

    // UART configure
    esp_err_t uart_param_config_status = uart_param_config(UART_NUM, &uart_config);
    if (uart_param_config_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(uart_param_config_status));
        return uart_param_config_status;
    }

    // UART pin setup
    esp_err_t uart_pin_init_status = uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (uart_pin_init_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(uart_pin_init_status));
        return uart_pin_init_status;
    }

    ESP_LOGI(TAG, "UART initialized successfully");
    return ESP_OK;
}

// LED control function/task
void led_control(void *pvParameters) {
    // Logging
    ESP_LOGI(TAG, "Starting LED Control.");

    // Setup GPIO
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    led_command_t recieved_command;
    TickType_t last_wake_time = xTaskGetTickCount();

    // Blinking loop
    while (1) {
        // Check for new commands & process
        if (xQueueReceive(cmd_queue, &recieved_command, 0) == pdTRUE) {
            if (xSemaphoreTake(led_mutex, portMAX_DELAY) == pdTRUE) {
                led_state.cur_command = recieved_command;
                switch(recieved_command) {
                    case LED_CMD_OFF:
                        led_state.is_blinking = false;
                        led_state.cur_level = false;
                        gpio_set_level(LED_PIN, 0);
                        ESP_LOGI(TAG, "TURNED LED OFF");
                        break;
                    case LED_CMD_ON:
                        led_state.is_blinking = false;
                        led_state.cur_level = true;
                        gpio_set_level(LED_PIN, 1);
                        ESP_LOGI(TAG, "TURNED LED ON");
                        break;
                    case LED_CMD_SLOW:
                        led_state.is_blinking = true;
                        led_state.delay = pdMS_TO_TICKS(1000);
                        ESP_LOGI(TAG, "ENABLED LED SLOW");
                        break;
                    case LED_CMD_FAST:
                        led_state.is_blinking = true;
                        led_state.delay = pdMS_TO_TICKS(100);
                        ESP_LOGI(TAG, "ENABLED LED FAST");
                        break;                    
                }
                xSemaphoreGive(led_mutex);
            }
        }

        // Handle blink logic
        if (led_state.is_blinking == true) {
            if (xSemaphoreTake(led_mutex, portMAX_DELAY) == pdTRUE) {
                led_state.cur_level = !led_state.cur_level;
                gpio_set_level(LED_PIN, led_state.cur_level);
                xSemaphoreGive(led_mutex);
            }
            vTaskDelayUntil(&last_wake_time, led_state.delay);
        }
        else {
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
        }
    }
}

// Command processing function/task
void command_handler(void *pvParameters) {
    char cmd_buf[64];
    int buf_index = 0;
    
    while(1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t*)&cmd_buf[buf_index], 1, 100);

        if (len > 0) {
            if (cmd_buf[buf_index] == '\n' || cmd_buf[buf_index] == '\r') {
                // Skip empty commands
                if (buf_index == 0) {
                    continue;
                }

                cmd_buf[buf_index] = '\0';
                buf_index = 0;
                uart_flush(UART_NUM);

                led_command_t cmd;

                if (strcmp(cmd_buf, "off") == 0) {
                    cmd = LED_CMD_OFF;
                }
                else if (strcmp(cmd_buf, "on") == 0) {
                    cmd = LED_CMD_ON;
                }
                else if (strcmp(cmd_buf, "fast") == 0) {
                    cmd = LED_CMD_FAST;
                }
                else if (strcmp(cmd_buf, "slow") == 0) {
                    cmd = LED_CMD_SLOW;
                }
                else {
                    ESP_LOGW(TAG, "UNKNOWN COMMAND: %s", cmd_buf);
                    continue;
                }

                // Send command to queue
                if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                    ESP_LOGE(TAG, "FAILED TO SEND COMMAND");
                }
            }
            else {
                buf_index++;

                // Overflow logic
                if (buf_index >= sizeof(cmd_buf) - 1) {
                    buf_index = 0;
                }
            }
        }
    }
}

// Main
void app_main(void) {
    // Logging
    ESP_LOGI(TAG, "Starting program");

    // UART Initialization
    esp_err_t init_uart_ret = init_uart();
    if (init_uart_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART");
        return;
    }

    // Initialization
    cmd_queue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(led_command_t));
    led_mutex = xSemaphoreCreateMutex();

    // Error checking
    if (cmd_queue == NULL || led_mutex == NULL) {
        ESP_LOGE(TAG, "FAILED TO CREATE SYNCHRONIZATION OBJECTS");
        return;
    }

    // Create tasks
    xTaskCreate(led_control, "LED CONTROLLER", LED_STACK_SIZE, NULL, LED_PRIORITY, NULL);   // LED Task
    xTaskCreate(command_handler, "COMMAND HANDLER", CMD_HNDLR_STACK_SIZE, NULL, CMD_HNDLR_PRIORITY, NULL); // cmd controller task
}