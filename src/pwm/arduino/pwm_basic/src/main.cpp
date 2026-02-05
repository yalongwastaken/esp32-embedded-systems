// Author: Anthony Yalong
// Description: Reads analog potentiometer value and controls LED brightness via PWM.
//              Updates every 50ms with serial monitoring for real-time feedback.

#include <Arduino.h>

// Hardware configuration
#define BAUDRATE 115200
#define LED_PIN 2                 // Built-in LED (ESP32-WROOM-32E)
#define POT_PIN A0                // Analog pin for a potentiometer
#define ANALOG_READ_RESOLUTION 12 // ESP32-WROOM-32E default value
#define ANALOG_WRITE_RESOLUTION 8 // ESP32-WROOM-32E default value
#define ANALOG_WRITE_FREQ 1000    // 1kHz PWM frequency
#define ADC_MAX 4095              // Max ADC value
#define PWM_MAX 255               // Max PWM value

// Timing
static unsigned long prev_time;

// Error passing
static bool system_error = false;

void setup()
{
    // Serial init
    Serial.begin(BAUDRATE);

    // Timing init
    prev_time = millis();

    // LED init
    if (!GPIO_IS_VALID_OUTPUT_GPIO(LED_PIN))
    {
        Serial.println("setup: invalid LED pin selection");
        system_error = true;
        return;
    }
    else
    {
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, 0);
        Serial.println("setup: LED initialized");
    }

    // Pot init
    if (!GPIO_IS_VALID_GPIO(POT_PIN))
    {
        Serial.println("setup: invalid potentiometer pin selection");
        system_error = true;
        return;
    }

    // Analog init
    analogReadResolution(ANALOG_READ_RESOLUTION);
    analogWriteResolution(ANALOG_WRITE_RESOLUTION);
    analogWriteFrequency(ANALOG_WRITE_FREQ);
    Serial.println("setup: analog initialized");
}

void loop()
{
    if (system_error)
    {
        return;
    }

    unsigned long cur_time = millis();
    if (cur_time - prev_time >= 50)
    {
        // update timing
        prev_time = cur_time;

        // Read potentiometer
        int pot_value = analogRead(POT_PIN);

        // PWM conversion
        int pwm_value = map(pot_value, 0, ADC_MAX, 0, PWM_MAX); // Linear mapping

        // Write
        analogWrite(LED_PIN, pwm_value);

        // Log update
        Serial.printf("loop: pot: %d, PWM: %d, brightness: %0.2f\n", pot_value, pwm_value, (pwm_value / 255.0) * 100);
    }
}