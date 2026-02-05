// Author: Anthony Yalong
// Description: UART-based LED control system using ESP-IDF and FreeRTOS

#include "uart_advanced_esp.h"
#include <string.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <ctype.h>

const char *TAG = "UART_ADVANCED (esp-idf)";

// freertos objects
QueueHandle_t uart_queue;
TaskHandle_t uart_event_task_handle;
QueueHandle_t led_queue;
TaskHandle_t led_control_task_handle;

// command structure
typedef struct {
    char *name;
    void (*handler)(const void *args);
    char *raw;
} command_t;

// led command enum
typedef enum {
    LED_ON,
    LED_OFF,
    LED_BLINK
} led_command_t;

// led state structure
typedef struct {
    led_command_t led_command;
    bool is_blinking;
    TickType_t delay;
} led_state_t;

void app_main() {
    esp_err_t ret;
    
    // initialize uart
    ret = init_uart();
    if (ret != ESP_OK) {
        halt_program();
    }

    // initialize led
    ret = init_led();
    if (ret != ESP_OK) {
        halt_program();
    }

    // create led queue
    led_queue = xQueueCreate(LED_QUEUE_LENGTH, sizeof(led_state_t));
    if (led_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LED queue");
        halt_program();
    }

    // create led control task
    xTaskCreate(
        led_control_task,
        "led_control_task",
        4096,
        NULL,
        7,
        &led_control_task_handle
    );

    // create uart event task
    xTaskCreate(
        uart_event_task,
        "uart_event_task",
        UART_TASK_STACK_DEPTH,
        NULL,
        UART_TASK_PRIO,
        &uart_event_task_handle
    );

    ESP_LOGI(TAG, "initialized uart!");

    // send welcome message
    const char* welcome_message = "\r\n=== ESP32 ESP-IDF UART ===\r\n"
                                  "Send commands to control the LED!\r\n"
                                  "Type 'help' for available commands\r\n\r\n";
    uart_write_bytes(UART_NUM, welcome_message, strlen(welcome_message));
}

void halt_program(void) {
    while (true) {
        ESP_LOGW(TAG, "system error!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t init_uart(void) {
    esp_err_t ret;

    // install uart driver
    ret = uart_driver_install(
        UART_NUM, 
        2 * UART_BUFFER_SIZE, 
        2 * UART_BUFFER_SIZE, 
        UART_QUEUE_SIZE,
        &uart_queue,
        0
    );
    if (ret != ESP_OK) {
        return ret;
    }

    // configure uart parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_BITS,
        .parity = UART_PARITY,
        .stop_bits = UART_STOP_BITS,
        .flow_ctrl = UART_FLW_CNTRL,
        .source_clk = UART_SRC_CLK
    };
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // configure uart pins
    ret = uart_set_pin(
        UART_NUM,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_RTS_PIN,
        UART_CTS_PIN
    );
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t init_led(void) {
    esp_err_t ret;

    // configure led pin
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&led_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // set initial led state to off
    ret = gpio_set_level(LED_PIN, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

void uart_event_task(void *pvParameters) {
    uart_event_t event;
    char cmd_buffer[128] = {0};
    int cmd_index = 0;
    uint8_t temp_buffer[UART_BUFF_SIZE];

    ESP_LOGI(TAG, "UART event task started");

    while (true) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY) == pdTRUE) {
            memset(temp_buffer, 0, UART_BUFF_SIZE);

            switch (event.type) {
                case UART_DATA: {
                    // read data into temp buffer
                    int len = uart_read_bytes(UART_NUM, temp_buffer, event.size, portMAX_DELAY);
                    
                    // process each byte
                    for (int i = 0; i < len; i++) {
                        char c = temp_buffer[i];
                        
                        // handle newline
                        if (c == '\n' || c == '\r') {
                            if (cmd_index == 0) {
                                continue;
                            }
                            
                            cmd_buffer[cmd_index] = '\0';
                            uart_write_bytes(UART_NUM, "\r\n", 2);
                            process(cmd_buffer, cmd_index);
                            
                            // reset for next command
                            cmd_index = 0;
                            memset(cmd_buffer, 0, sizeof(cmd_buffer));
                        }
                        // handle backspace
                        else if (c == '\b' || c == 127) {
                            if (cmd_index > 0) {
                                cmd_index--;
                                uart_write_bytes(UART_NUM, "\b \b", 3);
                            }
                        }
                        // handle printable characters
                        else if (c >= 32 && c <= 126) {
                            if (cmd_index < sizeof(cmd_buffer) - 1) {
                                cmd_buffer[cmd_index++] = c;
                                uart_write_bytes(UART_NUM, &c, 1);
                            } else {
                                const char *msg = "\r\nBuffer full!\r\n";
                                uart_write_bytes(UART_NUM, msg, strlen(msg));
                                cmd_index = 0;
                                memset(cmd_buffer, 0, sizeof(cmd_buffer));
                            }
                        }
                    }
                    break;
                }

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    cmd_index = 0;
                    memset(cmd_buffer, 0, sizeof(cmd_buffer));
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    cmd_index = 0;
                    memset(cmd_buffer, 0, sizeof(cmd_buffer));
                    break;

                case UART_BREAK:
                    break;
               
                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "UART parity error");
                    break;
                
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART frame error");
                    break;
                
                case UART_PATTERN_DET:
                    break;

                default:
                    break;
            }
        }
    }

    vTaskDelete(uart_event_task_handle);
}

void led_control_task(void *pvParameters) {
    led_state_t led_state = {
        .is_blinking = false,
        .delay = pdMS_TO_TICKS(0),
        .led_command = LED_OFF
    };
    TickType_t last_wake = xTaskGetTickCount();
    bool current_led_level = false;

    while (1) {
        // receive led command from queue
        if (xQueueReceive(led_queue, &led_state, pdMS_TO_TICKS(10)) == pdTRUE) {
            switch (led_state.led_command) {
                case LED_ON:
                    current_led_level = true;
                    gpio_set_level(LED_PIN, 1);
                    ESP_LOGI(TAG, "LED ON");
                    break;

                case LED_OFF:
                    current_led_level = false;
                    gpio_set_level(LED_PIN, 0);
                    ESP_LOGI(TAG, "LED OFF");
                    break;

                case LED_BLINK:
                    current_led_level = false;
                    gpio_set_level(LED_PIN, 0);
                    last_wake = xTaskGetTickCount();
                    break;

                default:
                    break;
            }
        }
        
        // handle led blinking
        TickType_t cur_time = xTaskGetTickCount();
        if (led_state.is_blinking && (cur_time - last_wake) >= led_state.delay) {
            current_led_level = !current_led_level;
            gpio_set_level(LED_PIN, current_led_level);
            ESP_LOGI(TAG, "LED %s", current_led_level ? "ON" : "OFF");
            last_wake = cur_time;
        }
    }
    
    vTaskDelete(led_control_task_handle);
}

void process(char *data, int len) {
    command_t cmd;
    cmd.raw = (char *)data;
    cmd.name = malloc(strlen((char *)data) + 1);
    if (cmd.name == NULL) {
        return;
    }

    // convert to lowercase
    for (int i = 0; cmd.raw[i]; i++) {
        cmd.name[i] = tolower(cmd.raw[i]);
    }
    cmd.name[strlen((char *)data)] = '\0';

    // match command to handler
    if (strcmp(cmd.name, "help") == 0) {
        cmd.handler = handler_help;
    }
    else if (strcmp(cmd.name, "on") == 0) {
        cmd.handler = handler_on;
    }
    else if (strcmp(cmd.name, "off") == 0) {
        cmd.handler = handler_off;
    }
    else if (strcmp(cmd.name, "blink") == 0) {
        cmd.handler = handler_blink;
    }
    else if (strcmp(cmd.name, "reset") == 0) {
        cmd.handler = handler_reset;
    }
    else {
        cmd.handler = handler_unknown;
    }
    
    cmd.handler(cmd.name);
    free(cmd.name);
}

void handler_help(const void *args) {
    const char *help_msg = 
        "Available commands: \r\n"
        "   help    - show this help\r\n"
        "   on      - turn on board LED\r\n"
        "   off     - turn off board LED\r\n"
        "   blink   - toggle blink on board LED\r\n"
        "   reset   - reset system\r\n";
    uart_write_bytes(UART_NUM, help_msg, strlen(help_msg));
}

void handler_on(const void *args) {
    led_state_t led_state = {
        .led_command = LED_ON,
        .is_blinking = false,
        .delay = 0
    };
    xQueueSend(led_queue, &led_state, portMAX_DELAY);
}

void handler_off(const void *args) {
    led_state_t led_state = {
        .led_command = LED_OFF,
        .is_blinking = false,
        .delay = 0
    };
    xQueueSend(led_queue, &led_state, portMAX_DELAY);
}

void handler_blink(const void *args) {
    led_state_t led_state = {
        .led_command = LED_BLINK,
        .is_blinking = true,
        .delay = pdMS_TO_TICKS(500)
    };
    xQueueSend(led_queue, &led_state, portMAX_DELAY);
}

void handler_reset(const void *args) {
    char reset_msg[] = "Resetting system in 3 seconds...\r\n";
    uart_write_bytes(UART_NUM, reset_msg, strlen(reset_msg));
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
}

void handler_unknown(const void *args) {
    const char *prefix = "Unknown command: '";
    const char *suffix = "'. Type 'help' for available commands.\r\n";
    
    uart_write_bytes(UART_NUM, prefix, strlen(prefix));
    if (args != NULL) {
        uart_write_bytes(UART_NUM, (char *) args, strlen((char *)args));
    }
    uart_write_bytes(UART_NUM, suffix, strlen(suffix));
}