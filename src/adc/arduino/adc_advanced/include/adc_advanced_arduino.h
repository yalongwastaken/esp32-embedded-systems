// Author: Anthony Yalong
// Description: Header file for `main.c`

#ifndef ADC_ADVANCED_ARDUINO_H
#define ADC_ADVANCED_ARDUINO_H

#include <Arduino.h>
#include <driver/gpio.h>

// project configuration
#define BAUDRATE 115200
#define POT_DELAY 50        // polling delay for potentiometer task
#define LIGHT_DELAY 100     // polling delay for light sensor task
#define SAMPLES 10          // smoothing samples count

// pin configuration
#define LED_POT GPIO_NUM_16
#define LED_LIGHT GPIO_NUM_17
#define POT_PIN GPIO_NUM_36         // adc1_0
#define LIGHT_PIN GPIO_NUM_39         // adc1_3

// pwm configuration
#define PWM_POT_CHANNEL 0
#define PWM_POT_FREQ 5000               // 5kHz
#define PWM_POT_BIT_RESOLUTION 8        // 8 bit resolution
#define PWM_POT_RESOLUTION 255          // 2^8 - 1
#define PWM_LIGHT_CHANNEL 1
#define PWM_LIGHT_FREQ 1000
#define PWM_LIGHT_BIT_RESOLUTION 10
#define PWM_LIGHT_RESOLUTION 1023

// adc configuration
#define ADC_BIT_RESOLUTION 12       // 12 bit resolution
#define ADC_RESOLUTION 4095         // 2^12 - 1
#define ADC_ATTENUATION ADC_11db

// light configuration
#define LIGHT_THRESHOLD_HIGH 2500
#define LIGHT_THRESHOLD_LOW 2000
#define LIGHT_HYSTERESIS (LIGHT_THRESHOLD_HIGH - LIGHT_THRESHOLD_LOW)
#define LIGHT_DEBOUNCE_TIME 1000

#endif  // ADC_ADVANCED_ARDUINO_H