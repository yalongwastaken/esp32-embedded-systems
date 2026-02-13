/**
 * @file main.cpp
 * @author Anthony Yalong
 * @brief 
 */

// includes
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"

// project configuration
#define BAUDRATE 115200
#define ADC1_PIN GPIO_NUM_36
#define ADC2_PIN GPIO_NUM_39

// adc configuration
#define ADC_ATTEN ADC_ATTEN_11db
#define ADC_BITWIDTH ADC_WIDTH_BIT_12

// sensor structure
typedef struct {
  gpio_num_t adc_pin;
  TickType_t delay_tick;
} sensor_config_t;

// display structure
typedef struct {
  gpio_num_t source;
  uint16_t raw_value;
}

void setup() {
  // serial
  Serial.begin(BAUDRATE);
  if (!Serial) delay(10);

  // adc initialization

}

void loop() {
}