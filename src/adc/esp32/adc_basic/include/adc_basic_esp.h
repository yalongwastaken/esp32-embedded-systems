// Author: Anthony Yalong
// Description: Header file for 'main.c'

#ifndef ADC_BASIC_ESP_H
#define ADC_BASIC_ESP_H

#include <esp_err.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// adc configuration
#define ADC_UNIT            ADC_UNIT_1
#define ADC_POT_CHANNEL     ADC_CHANNEL_0       // GPIO 36
#define ADC_LIGHT_CHANNEL   ADC_CHANNEL_3       // GPIO 39
#define ADC_BITWIDTH        ADC_BITWIDTH_12     // 12 bit (0-4095)
#define ADC_ATTEN           ADC_ATTEN_DB_12     // 0 - 3.9V

// task configuration
#define TASK_PRIO           3                   // low priority
#define TASK_STACK_DEPTH    2048                // small stack size
#define TASK_DELAY          1000                // 1 second delay

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
 * @brief FreeRTOS task that periodically reads and logs sensor data
 * 
 * Continuously reads raw ADC values, converts to voltage (if calibrated),
 * and logs results. Task configuration (channel, name, delay) is passed
 * via pvParameters as a sensor_task_config_t pointer.
 * 
 * @param pvParameters Pointer to sensor_task_config_t structure
 */
void log_sensor(void *pvParameters);

#endif // ADC_BASIC_ESP_H