// Author: Anthony Yalong
// Description: Simple, non-blocking, interactive blinking LED function using an Arduino based workspace.

// Include
#include <Arduino.h>
#include <vector>

// Defines
#define LED_PIN 2

// Pattern Struct
struct LED_Pattern {
  // variables
  std::vector<int> sequence;
  int index;
  unsigned long prev_time;

  // Constructor
  LED_Pattern(std::vector<int> seq): sequence(seq), index(0), prev_time(0) {};
};

// Variables
bool led_state = false;
std::vector<int> fast_seq = {100, 100};
std::vector<int> slow_seq = {1000, 1000};
LED_Pattern fast(fast_seq);
LED_Pattern slow(slow_seq);
LED_Pattern* cur_pattern = &slow;

void setup() {
  // Setup board
  pinMode(LED_PIN, OUTPUT);

  // Setup Serial
  Serial.begin(115200);
  Serial.println("Starting Blinking LED 3 (Arduino): ");
  Serial.println("Send `1` for fast. Send `2` for slow.");
}

// LED update logic
void update_led(LED_Pattern &pattern) {
  unsigned long cur_time = millis();

  if (cur_time - pattern.prev_time >= pattern.sequence[pattern.index]) {
    led_state = !led_state;
    pattern.prev_time = cur_time;
    digitalWrite(LED_PIN, led_state);
    pattern.index = (pattern.index + 1) % pattern.sequence.size();
    Serial.println(led_state ? "LED ON" : "LED OFF");
  }
}

// Main loop
void loop() {
  update_led(*cur_pattern);

  if (Serial.available()) {
    char command = Serial.read();

    switch (command) {
      case '1':
        cur_pattern = &fast;
        Serial.println("Pattern changed to `Fast`.");
        break;
      case '2':
        cur_pattern = &slow;
        Serial.println("Pattern changed to `Slow`.");
        break;
    }
  }
}