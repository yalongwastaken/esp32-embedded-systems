// Author: Anthony Yalong
// Description: Header file for UART-based LED control system using ESP-IDF and FreeRTOS

#ifndef UART_ADVANCED_ESP_H
#define UART_ADVANCED_ESP_H

#include <esp_err.h>
#include <stdint.h>
#include <driver/uart.h>
#include <driver/gpio.h>

// UART configuration
#define UART_NUM UART_NUM_0
#define UART_BUFFER_SIZE 1024
#define UART_QUEUE_SIZE 10
#define UART_BAUDRATE 115200
#define UART_DATA_BITS UART_DATA_8_BITS
#define UART_PARITY UART_PARITY_DISABLE
#define UART_STOP_BITS UART_STOP_BITS_1
#define UART_FLW_CNTRL UART_HW_FLOWCTRL_DISABLE
#define UART_SRC_CLK UART_SCLK_APB
#define UART_TX_PIN UART_PIN_NO_CHANGE
#define UART_RX_PIN UART_PIN_NO_CHANGE
#define UART_RTS_PIN UART_PIN_NO_CHANGE
#define UART_CTS_PIN UART_PIN_NO_CHANGE
#define UART_BUFF_SIZE 1024

// UART task configuration
#define UART_TASK_STACK_DEPTH 4096
#define UART_TASK_PRIO 10

// LED configuration
#define LED_PIN GPIO_NUM_2
#define LED_QUEUE_LENGTH 10

/**
 * @brief Halts program execution indefinitely on critical error
 */
void halt_program(void);

/**
 * @brief Initializes UART peripheral with configured parameters
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t init_uart(void);

/**
 * @brief Initializes LED GPIO pin for output
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t init_led(void);

/**
 * @brief FreeRTOS task that processes UART events from hardware queue
 * @param pvParameters Task parameters (unused)
 */
void uart_event_task(void *pvParameters);

/**
 * @brief FreeRTOS task that controls LED state based on commands from queue
 * @param pvParameters Task parameters (unused)
 */
void led_control_task(void *pvParameters);

/**
 * @brief Parses received command string and dispatches to appropriate handler
 * @param data Pointer to null-terminated command string
 * @param len Length of command string in bytes
 */
void process(char *data, int len);

/**
 * @brief Command handler that displays available commands
 * @param args Command arguments (unused)
 */
void handler_help(const void *args);

/**
 * @brief Command handler that turns LED on
 * @param args Command arguments (unused)
 */
void handler_on(const void *args);

/**
 * @brief Command handler that turns LED off
 * @param args Command arguments (unused)
 */
void handler_off(const void *args);

/**
 * @brief Command handler that starts LED blinking at 500ms intervals
 * @param args Command arguments (unused)
 */
void handler_blink(const void *args);

/**
 * @brief Command handler that performs system reset
 * @param args Command arguments (unused)
 */
void handler_reset(const void *args);

/**
 * @brief Command handler for unrecognized commands
 * @param args Pointer to the unrecognized command string
 */
void handler_unknown(const void *args);

#endif  // UART_ADVANCED_ESP_H