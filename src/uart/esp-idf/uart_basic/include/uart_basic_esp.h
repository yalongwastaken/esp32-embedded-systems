// Author: Anthony Yalong
// Description: Header file for `main.c`

#ifndef UART_BASIC_ESP_H
#define UART_BASIC_ESP_H

#include <esp_err.h>
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>

// UART Configuration
#define UART_NUM UART_NUM_0
#define UART_BAUDRATE 115200
#define UART_DATA_BITS UART_DATA_8_BITS
#define UART_PARITY UART_PARITY_DISABLE
#define UART_STOP_BITS UART_STOP_BITS_1
#define UART_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE
#define UART_SOURCE_CLK UART_SCLK_APB
#define UART_BUFF_SIZE (1024)
#define UART_QUEUE_SIZE 10
#define UART_TX_PIN UART_PIN_NO_CHANGE
#define UART_RX_PIN UART_PIN_NO_CHANGE
#define UART_RTS_PIN UART_PIN_NO_CHANGE
#define UART_CTS_PIN UART_PIN_NO_CHANGE
#define RX_TASK_STACK_SIZE 4096

/**
 * @brief Halts program execution on critical error
 */
void halt_program(void);

/**
 * @brief Initializes UART peripheral with configured parameters
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t init_uart(void);

/**
 * @brief FreeRTOS task that processes UART events from queue
 * @param pvParameters Task parameters (unused)
 */
void uart_event_task(void *pvParameters);

/**
 * @brief Parses received data and executes corresponding command
 * @param data Pointer to received data buffer
 * @param len Length of received data in bytes
 */
void process_cmd(char *data, int len);

#endif // UART_BASIC_ESP_H