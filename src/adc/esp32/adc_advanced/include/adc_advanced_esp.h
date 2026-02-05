// Author: Anthony Yalong
// Description: Header file for `main.c`

#ifndef ADC_ADVANCED_ESP_H
#define ADC_ADVANCED_ESP_H

#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <driver/ledc.h>

// project configuration
#define BAUDRATE 115200
#define DELAY_POT 50
#define DELAY_LIGHT 100

// task configuration
#define TASK_STACK_DEPTH 4096
#define TASK_PRIO 5

// gpio configuration
#define LED_POT GPIO_NUM_16
#define LED_LIGHT GPIO_NUM_17
#define PIN_POT ADC_CHANNEL_0     // adc1_0
#define PIN_LIGHT ADC_CHANNEL_3   // adc1_3
#define LEDC_CHANNEL_POT LEDC_CHANNEL_0
#define LEDC_CHANNEL_LIGHT LEDC_CHANNEL_1

// adc configuration
#define ADC_UNIT ADC_UNIT_1
#define ADC_BITWIDTH ADC_BITWIDTH_12    // 12 bit resolution
#define ADC_ATTEN ADC_ATTEN_DB_12       // 0 - 3.9V

// pwm configuration
#define LEDC_CLK_CFG LEDC_AUTO_CLK
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

// pwm potentiometer configuration
#define LEDC_TIMER_POT LEDC_TIMER_0
#define LEDC_FREQ_POT 5000
#define LEDC_RESOLUTION_POT LEDC_TIMER_8_BIT

// pwm light sensor configuration
#define LEDC_TIMER_LIGHT LEDC_TIMER_1
#define LEDC_FREQ_LIGHT 1000
#define LEDC_RESOLUTION_LIGHT LEDC_TIMER_10_BIT

// potentiometer task configuration
#define SAMPLES 10

// light sensor task configuration
#define LIGHT_THRESHOLD_HIGH 2500
#define LIGHT_THRESHOLD_LOW 2000
#define LIGHT_HYSTERESIS (LIGHT_THRESHOLD_HIGH - LIGHT_THRESHOLD_LOW)
#define LIGHT_DEBOUNCE_TIME 1000

/**
 * @brief Halts program execution with periodic error logging
 */
void halt_program(void);

/**
 * @brief Initializes ADC unit and configures sensor channels
 * @return ESP_OK on success, or ESP-IDF error code on failure
 */
esp_err_t init_adc(void);

/**
 * @brief Initializes PWM and configures sensor LEDs
 * @return ESP_OK on success, or ESP-IDF error code on failure
 */
esp_err_t init_pwm(void);

/**
 * @brief Task that reads potentiometer and controls LED brightness
 * @param pvParameters Task parameters (unused)
 */
void task_potentiometer(void *pvParameters);

/**
 * @brief Task that reads light sensor and controls LED with hysteresis
 * @param pvParameters Task parameters (unused)
 */
void task_light_sensor(void *pvParameters);

#endif  // ADC_ADVANCED_ESP_H