// Author: Anthony Yalong
// Description: Simple, non-blocking blinking LED for the ESP32 developed using the Arduino framework.

// Includes
#include <Arduino.h>

// Define
#define LED_PIN 2
#define LED_DELAY 1000

// State Variables
unsigned long previousMillis = millis();
bool led_state = false;

void setup() {
  // Logging
  Serial.begin(115200);
  Serial.println("Initializeing LED Blink (Non-blocking)");

  // Initialization
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_state);
}

void loop() {
  unsigned long currentMillis = millis();

  // Update condition
  if (currentMillis - previousMillis >= LED_DELAY) {
    previousMillis = currentMillis;
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state);
    Serial.print("Blinking LED 2 (Arduino): ");
    Serial.println(led_state ? "LED ON" : "LED OFF");
  }
}