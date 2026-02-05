// Author: Anthony Yalong
// Description: FreeRTOS-based UART command interface with support for system info,
//              status checking, echo functionality, and system reset using UART event
//              queue for asynchronous data reception with overflow protection

#include "uart_basic_esp.h"
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <ctype.h>
#include <esp_timer.h>

const char *TAG = "UART_BASIC (esp-idf)";

// freertos objects
TaskHandle_t uart_rx_task_handle = NULL;
QueueHandle_t uart_queue;

void app_main() {
    esp_err_t ret;

    // initialize uart
    ret = init_uart();
    if (ret != ESP_OK) {
        halt_program();
    }

    // create uart event task
    xTaskCreate(
        uart_event_task, 
        "uart_event_task",
        RX_TASK_STACK_SIZE,
        NULL,
        1,
        &uart_rx_task_handle
    );

    ESP_LOGI(TAG, "UART initialization complete.");

    // send welcome message
    const char* welcome_message = "\r\n=== ESP32 ESP-IDF UART Example ===\r\n"
                                  "Send commands to control the system\r\n"
                                  "Type 'help' for available commands\r\n\r\n";
    uart_write_bytes(UART_NUM, welcome_message, strlen(welcome_message));
}

void halt_program(void) {
    while (1) {
        ESP_LOGW(TAG, "System Error!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t init_uart(void) {
    esp_err_t ret;

    // install uart driver
    ret = uart_driver_install(UART_NUM, 2 * UART_BUFF_SIZE, 2 * UART_BUFF_SIZE, UART_QUEUE_SIZE, &uart_queue, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    // configure uart parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_BITS,
        .parity = UART_PARITY,
        .stop_bits = UART_STOP_BITS,
        .flow_ctrl = UART_FLOW_CTRL,
        .source_clk = UART_SOURCE_CLK,
    };

    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // configure uart pins
    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_CTS_PIN);
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
                            process_cmd(cmd_buffer, cmd_index);
                            
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

    vTaskDelete(uart_rx_task_handle);
}

void process_cmd(char *data, int len) {
    // convert to lowercase
    for (int i = 0; i < len; i++) {
        data[i] = tolower(data[i]);
    }

    // match command and execute
    if (strcmp(data, "help") == 0) {
        const char *help_msg = 
            "Available commands: \r\n"
            "   help        - show this help\r\n"
            "   info        - system information\r\n"
            "   status      - system status\r\n"
            "   reset       - reset system\r\n"
            "   echo <text> - echo back <text>\r\n";
        uart_write_bytes(UART_NUM, help_msg, strlen(help_msg));
    }
    else if (strcmp(data, "info") == 0) {
        char info_buffer[512];
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);

        snprintf(
            info_buffer, sizeof(info_buffer),
            "System Information:\r\n"
            "   Chip: %s\r\n"
            "   Revision: %d\r\n"
            "   IDF Version: %s\r\n"
            "   Free Heap: %lu bytes\r\n"
            "   Uptime: %lld seconds\r\n",
            CONFIG_IDF_TARGET,
            chip_info.revision,
            esp_get_idf_version(),
            (unsigned long)esp_get_free_heap_size(),
            (long long)(esp_timer_get_time() / 1000000)
        );
        uart_write_bytes(UART_NUM, info_buffer, strlen(info_buffer));
    }
    else if (strcmp(data, "status") == 0) {
        char status_buffer[256];

        snprintf(
            status_buffer, sizeof(status_buffer),
            "System Status: OK\r\n"
            "UART PORT: %d\r\n"
            "Baudrate: %d\r\n"
            "Task Priority: %lu\r\n",
            UART_NUM,
            UART_BAUDRATE,
            (unsigned long)uxTaskPriorityGet(NULL)
        );
        uart_write_bytes(UART_NUM, status_buffer, strlen(status_buffer));
    }
    else if (strcmp(data, "reset") == 0) {
        const char *reset_msg = "Resetting system in 3 seconds...\r\n";
        uart_write_bytes(UART_NUM, reset_msg, strlen(reset_msg));
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    else if (strncmp(data, "echo ", 5) == 0) {
        if (len > 5) {
            char echo_buffer[256];
            snprintf(
                echo_buffer, sizeof(echo_buffer),
                "ECHO: %s\r\n",
                data + 5
            );
            uart_write_bytes(UART_NUM, echo_buffer, strlen(echo_buffer));
        } 
        else {
            const char *empty_msg = "ECHO: <empty>\r\n";
            uart_write_bytes(UART_NUM, empty_msg, strlen(empty_msg));
        }
    }
    else {
        char error_buffer[128];
        snprintf(
            error_buffer, sizeof(error_buffer),
            "UNKNOWN COMMAND: '%s'. Type 'help' for command list.\r\n",
            data
        );
        uart_write_bytes(UART_NUM, error_buffer, strlen(error_buffer));
    }
}