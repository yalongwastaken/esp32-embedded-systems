// Author: Anthony Yalong
// Description: Header file for `main.c`

#ifndef ADC_BASIC_ARDUINO_H
#define ADC_BASIC_ARDUINO_H

#include <Arduino.h>

// project configuration
#define BAUDRATE 115200
#define DELAY 500

// pin configuration
#define POT_PIN 36      // ADC1_0
#define LIGHT_PIN 39    // ADC1_3

// adc configuration
#define ADC_ATTENUATION ADC_11db         // 0.0-3.9V range    
#define ADC_BIT_RESOLUTION 12            // 12 bit resolution
#define ADC_RESOLUTION 4095.0f           // 12 bit resolution
#define ADC_REF_VOLT 3.3f                // reference voltage (3.3V)
#define ADC_VOLT_PER_STEP (ADC_REF_VOLT / ADC_RESOLUTION)

#endif // ADC_BASIC_ARDUINO_H