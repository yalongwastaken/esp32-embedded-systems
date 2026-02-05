// Author: Anthony Yalong
// Description: Header file for `main.c`

#ifndef I2C_LCD_BASIC_ARDUINO_H
#define I2C_LCD_BASIC_ARDUINO_H

// imports
#include <Wire.h>
#include <Arduino.h>
#include <driver/gpio.h>

// file configuration
#define BAUDRATE 115200
#define CLOCK_SPEED_DEFAULT 100000
#define SCAN_DELAY 5000

// i2c configuration
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_START_ADDR 0x08
#define I2C_END_ADDR 0x77


#endif  // I2C_LCD_BASIC_ARDUINO_H