// Author: Anthony Yalong
// Description: Bare-bones switch-LED integration using ESP32 (Arduino framework). Reads a push-button input and controls an LED output via GPIO

#include <Arduino.h>

// GPIO definitions
const int LED_PIN = 2;
const int SWITCH_PIN = 4;

void setup() {
  // Setup serial
  Serial.begin(115200);
  Serial.println("Switch (Arduino): Starting program.");

  // LED init
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Swtich init
  pinMode(SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  // Read switch
  bool is_pressed = !digitalRead(SWITCH_PIN);

  // Update LED
  digitalWrite(LED_PIN, is_pressed ? HIGH : LOW);

  // Logging
  static bool state = false;
  if (state != is_pressed) {
    state = is_pressed; 
    Serial.println("Switch (Arduino): " + String(is_pressed ? "LED ON" : "LED OFF"));
  }

  // Delay
  delay(10);
}