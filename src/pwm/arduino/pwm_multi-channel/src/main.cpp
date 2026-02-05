// Author: Anthony Yalong
// Description: Controls an RGB LED using ESP32 hardware PWM (LEDC).
//              Generates a new random color every 1 second using esp_random(),
//              with configurable timing and resolution constants.

#include <Arduino.h>
#include <driver/ledc.h>
#include <esp_random.h>

#define BAUDRATE 115200

// LED definitions
#define RED_PIN 23
#define GREEN_PIN 22
#define BLUE_PIN 21

// PWM Channels
#define RED_CHANNEL LEDC_CHANNEL_0
#define GREEN_CHANNEL LEDC_CHANNEL_1
#define BLUE_CHANNEL LEDC_CHANNEL_2

// PWM
#define PWM_FREQ 5000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_RANGE 256 // 2^8

// Timing
#define LED_DELAY_MS 1000
static unsigned long prev_time;

// Error logging
static bool system_error = false;

// Function definitions
esp_err_t setup_pwm_channel(int pin, ledc_channel_t channel);
void change_rgb_led(ledc_channel_t r, ledc_channel_t g, ledc_channel_t b);

void setup() {
  // Setup Serial communication
  Serial.begin(BAUDRATE);
  delay(100);     // Initialization delay

  // Error logging
  esp_err_t ret;

  // Setup PWM timer
  ledc_timer_config_t timer_config = {
    .speed_mode = SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ret = ledc_timer_config(&timer_config);
  if (ret != ESP_OK) {
    Serial.printf("setup: failed to initialize %s\n", esp_err_to_name(ret));
    system_error = true;
    return;
  }

  // Setup RGB Channels
  ret = setup_pwm_channel(RED_PIN, RED_CHANNEL);
  if (ret != ESP_OK) {
    Serial.printf("setup: failed to initialize %s\n", esp_err_to_name(ret));
    system_error = true;
    return;
  }

  ret = setup_pwm_channel(GREEN_PIN, GREEN_CHANNEL);
  if (ret != ESP_OK) {
    Serial.printf("setup: failed to initialize %s\n", esp_err_to_name(ret));
    system_error = true;
    return;
  }

  ret = setup_pwm_channel(BLUE_PIN, BLUE_CHANNEL);
  if (ret != ESP_OK) {
    Serial.printf("setup: failed to initialize %s\n", esp_err_to_name(ret));
    system_error = true;
    return;
  }

  // Timing
  prev_time = millis();
}

void loop() {
  if (system_error) {
    return;
  }

  unsigned long cur_time = millis();
  if (cur_time - prev_time >= LED_DELAY_MS) {
    prev_time = cur_time;
    change_rgb_led(RED_CHANNEL, GREEN_CHANNEL, BLUE_CHANNEL);
  }
}

esp_err_t setup_pwm_channel(int pin, ledc_channel_t channel) {
  ledc_channel_config_t channel_config = {
    .gpio_num = pin,
    .speed_mode = SPEED_MODE,
    .channel = channel,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  return ledc_channel_config(&channel_config);
}

void change_rgb_led(ledc_channel_t r, ledc_channel_t g, ledc_channel_t b) {
  uint32_t red = esp_random() % PWM_RANGE;    // 0-255
  uint32_t green = esp_random() % PWM_RANGE;  // 0-255  
  uint32_t blue = esp_random() % PWM_RANGE;   // 0-255

  ledc_set_duty(SPEED_MODE, r, red);
  ledc_set_duty(SPEED_MODE, g, green);
  ledc_set_duty(SPEED_MODE, b, blue);

  ledc_update_duty(SPEED_MODE, r);
  ledc_update_duty(SPEED_MODE, g);
  ledc_update_duty(SPEED_MODE, b);

  Serial.printf("change_rgb_led: color change to (%lu, %lu, %lu)\n", red, green, blue);
}