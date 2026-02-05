// Author: Anthony Yalong
// Description: Non-blocking serial echo tool with message length reporting

// Hardware configuration
#define BAUDRATE 115200

// Logging
const char ECHO_PREFIX[] = "ECHO: ";

#include <Arduino.h>

void setup() {
    // Initialize UART
    Serial.begin(BAUDRATE);
    
    // Wait for setup
    while (!Serial) delay(10);      // Blocking delay

    // Clear existing buffers
    Serial.flush();
    while (Serial.available()) Serial.read();

    // Initial message
    Serial.println("===ESP32 Serial Echo Tool===");
    Serial.println("Ready to echo your input?");
    Serial.print("Baudrate: ");
    Serial.println(BAUDRATE);
    Serial.println("Type characters & press `Enter`:");
}

void loop() {
    // Read incoming data
    if (Serial.available() > 0) {
        String serial_data = Serial.readStringUntil('\n');
        serial_data.trim();

        if (serial_data.length() > 0) {
            Serial.printf("%s'%s' (Length: %d chars)\n", 
            ECHO_PREFIX, serial_data.c_str(), serial_data.length());
        }
    }
}