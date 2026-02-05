// Author: Anthony Yalong
// Description: Basic ADC demonstration reading potentiometer and light sensor values

#include "adc_basic_arduino.h"

// logging
const char* TAG = "adc_basic (arduino)";
unsigned long last_sample_time;

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) delay(10);

  // initialize adc
  analogSetAttenuation(ADC_ATTENUATION);
  analogReadResolution(ADC_BIT_RESOLUTION);

  // initialize timing
  last_sample_time = millis();

  Serial.printf("%s: setup complete\n", TAG);
  Serial.printf("%s: starting measurements:\n", TAG);
  Serial.printf("%s: TIME (ms)\tPOT_RAW\tPOT_VOLTAGE\tLIGHT_RAW\tLIGHT_VOLTAGE\n", TAG);
}

void loop() {
  unsigned long cur_time = millis();
  if (cur_time - last_sample_time >= DELAY) {
    // read values
    int pot_raw = analogRead(POT_PIN);
    int light_raw = analogRead(LIGHT_PIN);

    // convert
    float pot_voltage = pot_raw * ADC_VOLT_PER_STEP;
    float light_voltage = light_raw * ADC_VOLT_PER_STEP;

    Serial.printf("%s: %lu\t%d\t%0.4f\t%d\t%0.4f\n", 
      TAG,
      millis(),
      pot_raw,
      pot_voltage,
      light_raw,
      light_voltage
    );

    last_sample_time = cur_time;
  }
}
