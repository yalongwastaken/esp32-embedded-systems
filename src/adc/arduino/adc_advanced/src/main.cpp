// Author: Anthony Yalong
// Description: Arduino-based ADC and PWM control system with dual sensor monitoring.
//              Potentiometer controls LED brightness via moving average filtering.
//              Light sensor controls LED with hysteresis and debouncing.
//              Implements cooperative multitasking using millis()-based scheduling.

#include "adc_advanced_arduino.h"

// logging
const char *TAG = "adc_advanced (arduino)";

// timing
unsigned long pot_last_wake;
unsigned long light_last_wake;

// potentiometer sampling
int pot_readings[SAMPLES];
int pot_read_index;
int pot_total;
int pot_average;

// light sensor debouncing
bool led_state;
unsigned long last_toggle;

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) delay(10);

  // initialize LEDs
  pinMode(LED_POT, OUTPUT);
  pinMode(LED_LIGHT, OUTPUT);
  digitalWrite(LED_POT, 0);
  digitalWrite(LED_LIGHT, 0);

  // initialize PWM
  ledcSetup(PWM_POT_CHANNEL, PWM_POT_FREQ, PWM_POT_BIT_RESOLUTION);
  ledcAttachPin(LED_POT, PWM_POT_CHANNEL);
  ledcSetup(PWM_LIGHT_CHANNEL, PWM_POT_FREQ, PWM_POT_BIT_RESOLUTION);
  ledcAttachPin(LED_LIGHT, PWM_LIGHT_CHANNEL);

  // initialize ADC
  analogSetAttenuation(ADC_ATTENUATION);
  analogReadResolution(ADC_BIT_RESOLUTION);

  // initialize potentiometer sampling
  for (int i = 0; i < SAMPLES; i++) {
    pot_readings[i] = 0;
  }
  pot_read_index = 0;
  pot_total = 0;
  pot_average = 0;

  // initialize light sensor debouncing
  led_state = false;
  last_toggle = 0;

  // initialize timing
  pot_last_wake = millis();
  light_last_wake = millis();

  // logging
  Serial.printf("%s: setup complete\n", TAG);
  Serial.printf("%s: pot task interval: %dms\n", TAG, POT_DELAY);
  Serial.printf("%s: light task interval: %dms\n", TAG, LIGHT_DELAY);
}

void loop() {
  unsigned long cur_time = millis();

  // pot task 
  if (cur_time - pot_last_wake >= POT_DELAY) {
    // sampling
    pot_total -= pot_readings[pot_read_index];
    pot_readings[pot_read_index] = analogRead(POT_PIN);
    pot_total += pot_readings[pot_read_index];
    pot_read_index = (pot_read_index + 1) % SAMPLES;
    pot_average = pot_total / SAMPLES;

    // mapping
    int pot_pwm_val = map(pot_average, 0, ADC_RESOLUTION, 0, PWM_POT_RESOLUTION);
    pot_pwm_val = constrain(pot_pwm_val, 0 , PWM_POT_RESOLUTION);

    // updating
    ledcWrite(PWM_POT_CHANNEL, pot_pwm_val);
    pot_last_wake = cur_time;

    // logging
    Serial.printf("%s: pot task: raw=%d avg=%d pwm=%d\n", TAG, pot_readings[(pot_read_index - 1 + SAMPLES) % SAMPLES], pot_average, pot_pwm_val);
  }

  if (cur_time - light_last_wake >= LIGHT_DELAY) {
    int light_level = analogRead(LIGHT_PIN);

    // debouncing
    if (
      !led_state &&
      light_level < LIGHT_THRESHOLD_LOW &&
      (cur_time - last_toggle >= LIGHT_DEBOUNCE_TIME)
    ) {
      led_state = true;
      last_toggle = cur_time;
      Serial.printf("%s: light task: LED ON - dark detected\n", TAG);
    }
    else if (
      led_state &&
      light_level > LIGHT_THRESHOLD_HIGH &&
      (cur_time - last_toggle >= LIGHT_DEBOUNCE_TIME)
    ) {
      led_state = false;
      last_toggle = cur_time;
      Serial.printf("%s: light task: LED OFF - light detected\n", TAG);
    }

    // pwm
    int light_pwm_val = 0;
    if (led_state) {
      light_pwm_val = map(light_level, 0, LIGHT_THRESHOLD_LOW, PWM_LIGHT_RESOLUTION, 100);
      light_pwm_val = constrain(light_pwm_val, 100, PWM_LIGHT_RESOLUTION);
    }

    // updating
    ledcWrite(PWM_LIGHT_CHANNEL, light_pwm_val);
    light_last_wake = cur_time;

    // logging
    Serial.printf("%s: light task: level=%d state=%s pwm=%d\n", TAG, light_level, led_state ? "ON" : "OFF", light_pwm_val);
  }
}