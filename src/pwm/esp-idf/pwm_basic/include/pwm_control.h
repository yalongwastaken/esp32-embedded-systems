// Author: Anthony Yalong
// Description: Reads analog potentiometer value and controls LED brightness via PWM.
//              Updates periodically with serial monitoring for real-time feedback.

#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <esp_log.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// GPIO Configuration
#define LED_NUM GPIO_NUM_2

// PWM Configuration
#define PWM_TIMER LEDC_TIMER_0
#define PWM_TIMER_CONFIG LEDC_AUTO_CLK
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define PWM_FREQ 1000
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TASK_DELAY 100

// ADC Configuration
#define ADC_CHANNEL ADC_CHANNEL_0       // GPIO36
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12   // 0.0V - 3.3V

// PWM Mapping Configuration
#define IN_MIN 0
#define IN_MAX ((1 << ADC_WIDTH) - 1)
#define OUT_MIN 0
#define OUT_MAX ((1 << PWM_RESOLUTION) - 1)

// Task Configuration
#define STACK_DEPTH 2048
#define TASK_PRIO 1

// Queue Configuration
#define QUEUE_SIZE 10
#define QUEUE_ITEM_TYPE esp_err_t
#define QUEUE_PRIO TASK_PRIO+1

/**
 * @brief Send an error to the system error queue.
 *
 * Logs the error source and ESP-IDF error code, then pushes it to the
 * global error queue so the error monitor task can act on it.
 *
 * @param source Human-readable string identifying where the error occurred.
 * @param err    ESP-IDF error code to report.
 */
void queue_error(char *source, esp_err_t err);

/**
 * @brief Initialize ADC hardware for reading potentiometer input.
 *
 * Configures ADC width and channel attenuation. Reports any errors
 * via queue_error().
 */
void init_adc(void);

/**
 * @brief Initialize PWM hardware for controlling LED brightness.
 *
 * Configures LEDC timer and channel for PWM output. Reports errors
 * via queue_error().
 */
void init_pwm(void);

/**
 * @brief Update PWM duty cycle for LED brightness.
 *
 * Clamps the duty value to the maximum allowed by PWM_RESOLUTION,
 * sets the duty, and updates the PWM hardware. Reports any errors
 * via queue_error().
 *
 * @param duty Desired duty cycle (0 .. 2^PWM_RESOLUTION - 1).
 */
void set_duty(uint32_t duty);

/**
 * @brief Read raw ADC value from configured channel.
 *
 * @return Raw ADC reading as a 12-bit integer (0–4095).
 */
uint32_t read_raw_adc(void);

/**
 * @brief Map a value from one range to another.
 *
 * Performs linear scaling from [in_min, in_max] to [out_min, out_max].
 *
 * @param value   Input value to be mapped.
 * @param in_min  Minimum of input range.
 * @param in_max  Maximum of input range.
 * @param out_min Minimum of output range.
 * @param out_max Maximum of output range.
 *
 * @return Mapped output value.
 */
uint32_t map_value(uint32_t value, uint32_t in_min, uint32_t in_max, 
    uint32_t out_min, uint32_t out_max);

/**
 * @brief Main control task.
 *
 * Reads potentiometer values via ADC, maps them to a PWM duty range,
 * and updates LED brightness accordingly. Runs in a continuous loop
 * with periodic delays.
 *
 * @param pvParameters Unused task parameter.
 */
void main_task(void *pvParameters);

/**
 * @brief System error monitor task.
 *
 * Waits for errors from the global error queue. When an error is received,
 * it logs the failure, cleans up the main task, and halts the system.
 *
 * @param pvParameters Unused task parameter.
 */
void error_monitor(void *pvParameters);

#endif