/**
 * @file i2c-lcd_basic_esp.h
 * @author Anthony Yalong
 * @brief I2C master initialization and device scanning utilities for ESP32
 */

#ifndef I2C_LCD_BASIC_ESP_H
#define I2C_LCD_BASIC_ESP_H

// includes
#include "esp_err.h"
#include "driver/i2c.h"

// project configuration
#define BAUDRATE 115200

// i2c configuration
#define I2C_NUM I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_START_ADDR 0x08
#define I2C_END_ADDR 0x77
#define I2C_MASTER_FREQ_HZ 100000  // standard mode
#define I2C_MASTER_RX_BUFF_SIZE 0
#define I2C_MASTER_TX_BUFF_SIZE 0
#define I2C_ACK_CHECK_EN true
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_TASK_ADDRESS_PROBE_DELAY_MS 10
#define I2C_TASK_DELAY_MS 5000
#define I2C_TASK_STACK_DEPTH 4096
#define I2C_TASK_PRIO 5

/**
 * @brief Halts program execution with periodic error logging
 */
void halt_program(void);

/**
 * @brief Initialize I2C master interface
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_init(void);

/**
 * @brief Probe a specific I2C address for device presence
 * @param addr 7-bit I2C device address to probe (0x08-0x77)
 * @return esp_err_t ESP_OK if device responds, ESP_ERR_TIMEOUT if no device found
 */
esp_err_t i2c_probe_address(uint8_t addr);

/**
 * @brief FreeRTOS task to scan I2C bus for connected devices
 * @param pvParameters Task parameters (unused, can be NULL)
 */
void i2c_scanner_task(void *pvParameters);

#endif  // I2C_LCD_BASIC_ESP_H