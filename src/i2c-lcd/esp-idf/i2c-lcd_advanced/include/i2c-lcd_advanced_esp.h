/**
 * @file i2c_lcd_advanced_esp.h
 * @author Anthony Yalong
 * @brief I2C LCD with potentiometer-controlled PWM LED brightness
 */

#ifndef I2C_LCD_ADVANCED_ESP_H
#define I2C_LCD_ADVANCED_ESP_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

// general configuration
#define BAUDRATE 115200

// i2c configuration
#define I2C_NUM I2C_NUM_0
#define I2C_MODE I2C_MODE_MASTER
#define I2C_SDA_PIN_NUM GPIO_NUM_21
#define I2C_SCL_PIN_NUM GPIO_NUM_22
#define I2C_MASTER_CLK_SPEED_HZ 100000  // standard mode
#define I2C_MASTER_RX_BUFF_SIZE 0
#define I2C_MASTER_TX_BUFF_SIZE 0
#define I2C_TIMEOUT_MS 1000

// lcd configuration
#define LCD_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define LCD_CMD_CLEAR           0x01
#define LCD_CMD_HOME            0x02
#define LCD_CMD_ENTRY_MODE      0x04
#define LCD_CMD_DISPLAY_CTRL    0x08
#define LCD_CMD_FUNCTION_SET    0x20
#define LCD_CMD_DDRAM_ADDR      0x80
#define LCD_ENTRY_LEFT          0x02
#define LCD_DISPLAY_ON          0x04
#define LCD_4BIT_MODE           0x00
#define LCD_2_LINE              0x08
#define LCD_5x8_DOTS            0x00
#define LCD_BACKLIGHT           0x08
#define LCD_NO_BACKLIGHT        0x00
#define LCD_EN                  0x04
#define LCD_RW                  0x02
#define LCD_RS                  0x01

// adc configuration
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_0
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_10    // 1024

// pwm configuration
#define PWM_TIMER LEDC_TIMER_0
#define PWM_CLK_CFG LEDC_AUTO_CLK
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_FREQ 5000       // 5000Hz
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0

// potentiometer task configuration
#define POT_LED_PIN GPIO_NUM_2
#define POT_SAMPLING 10
#define POT_TASK_DELAY_MS 500
#define POT_TASK_STACK_DEPTH 4096
#define POT_TASK_PRIO 5

// lcd task configuration
#define LCD_TASK_DELAY_MS 100
#define LCD_TASK_STACK_DEPTH 4096
#define LCD_TASK_PRIO 5

// lcd structure
typedef struct {
    i2c_port_t i2c_port;
    uint8_t addr;
    uint8_t cols;
    uint8_t rows;
    uint8_t backlight_state;
} lcd_handle_t;

// potentiometer state structure
typedef struct {
    uint32_t samples[POT_SAMPLING];
    uint32_t total;
    uint8_t index;
    uint8_t brightness_pct;
} pot_state_t;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Halts program execution with periodic error logging
 */
void halt_program(void);

// ============================================================================
// I2C Functions
// ============================================================================

/**
 * @brief Initialize I2C master interface
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_init(void);

// ============================================================================
// LCD Functions
// ============================================================================

/**
 * @brief Initialize LCD display
 * @param lcd Pointer to LCD handle structure
 * @param i2c_port I2C port number
 * @param addr I2C address of LCD
 * @param cols Number of LCD columns
 * @param rows Number of LCD rows
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_init(lcd_handle_t *lcd, i2c_port_t i2c_port, uint8_t addr, uint8_t cols, uint8_t rows);

/**
 * @brief Clear LCD display
 * @param lcd Pointer to LCD handle structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_clear(lcd_handle_t *lcd);

/**
 * @brief Set cursor position on LCD
 * @param lcd Pointer to LCD handle structure
 * @param col Column position
 * @param row Row position
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_set_cursor(lcd_handle_t *lcd, uint8_t col, uint8_t row);

/**
 * @brief Control LCD backlight
 * @param lcd Pointer to LCD handle structure
 * @param state true to turn on, false to turn off
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_backlight(lcd_handle_t *lcd, bool state);

/**
 * @brief Print string to LCD
 * @param lcd Pointer to LCD handle structure
 * @param str Null-terminated string to print
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_print(lcd_handle_t *lcd, const char *str);

/**
 * @brief Print formatted string to LCD
 * @param lcd Pointer to LCD handle structure
 * @param format Format string
 * @param ... Variable arguments
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_printf(lcd_handle_t *lcd, const char *format, ...);

/**
 * @brief Write byte to LCD in 4-bit mode
 * @param lcd Pointer to LCD handle structure
 * @param data Byte to write
 * @param mode Register select mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_write_byte(lcd_handle_t *lcd, uint8_t data, uint8_t mode);

/**
 * @brief Write nibble to LCD
 * @param lcd Pointer to LCD handle structure
 * @param nibble Nibble to write
 * @param mode Register select mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_write_nibble(lcd_handle_t *lcd, uint8_t nibble, uint8_t mode);

/**
 * @brief Send command to LCD
 * @param lcd Pointer to LCD handle structure
 * @param cmd Command byte
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_send_command(lcd_handle_t *lcd, uint8_t cmd);

/**
 * @brief Send data to LCD
 * @param lcd Pointer to LCD handle structure
 * @param data Data byte
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_send_data(lcd_handle_t *lcd, uint8_t data);

/**
 * @brief Pulse enable bit to latch data
 * @param lcd Pointer to LCD handle structure
 * @param data Data byte
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t lcd_pulse_enable(lcd_handle_t *lcd, uint8_t data);

// ============================================================================
// ADC Functions
// ============================================================================

/**
 * @brief Initialize ADC for potentiometer reading
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t adc_init(void);

// ============================================================================
// PWM Functions
// ============================================================================

/**
 * @brief Initialize PWM for LED control
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t pwm_init(void);

// ============================================================================
// Task Functions
// ============================================================================

/**
 * @brief Potentiometer reading and PWM control task
 * @param pvParameters Task parameters (unused)
 */
void pot_task(void *pvParameters);

/**
 * @brief LCD display update task
 * @param pvParameters Task parameters (unused)
 */
void lcd_task(void *pvParameters);

#endif  // I2C_LCD_ADVANCED_ESP_H