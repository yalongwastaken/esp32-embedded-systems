// Author: Anthony Yalong
// Description: Header file for `main.c`

#ifndef I2C_LCD_ADVANCED_ARDUINO_H
#define I2C_LCD_ADVANCED_ARDUINO_H

// imports
#include <Wire.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <driver/gpio.h>

// project configuration
#define BAUDRATE 115200

// i2c configuration
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_START_ADDR 0x08
#define I2C_END_ADDR 0x77
#define I2C_DEFAULT_CLOCK 100000

// lcd configuration
#define LCD_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define LCD_DELAY 100

// adc configuration
#define PIN_POT GPIO_NUM_36 // adc1_0
#define ADC_ATTENUATION ADC_11db    // 0 - 3.9V
#define ADC_RESOLUTION 10

// pwm configuration
#define PIN_LED GPIO_NUM_2
#define PWM_CHANNEL 0
#define PWM_BIT_RESOLUTION 8        // 8 bit resolution.
#define PWM_FREQUENCY 5000          // 5k Hz

// potentiometer task configuration
#define SAMPLING 10
#define POT_TASK_DELAY 500

#endif  // I2C_LCD_ADVANCED_ARDUINO_H