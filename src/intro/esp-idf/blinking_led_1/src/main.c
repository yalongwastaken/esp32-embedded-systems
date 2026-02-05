// Author: Anthony Yalong
// Description: Simple blinking LED for the ESP32 developed using the Arduino framework.

// Includes
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_system.h>

// Defines
#define LED_GPIO GPIO_NUM_2
#define LED_DELAY 1000

// Logging
static const char *TAG = "Blinking LED 1 (ESP-IDF)";

// LED blink task
void led_blink(void *pvParameters) {
    // Setup
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),    // Bitmask of pins to configure
        .mode = GPIO_MODE_OUTPUT,              // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE         // Disable interrupts
    };

    gpio_config(&gpio_conf);
    bool led_state = false;

    // Timing
    TickType_t last_wake_time = xTaskGetTickCount();

    // Logic loop
    while (1) {
        // toggle LED state
        led_state = !led_state;
        gpio_set_level(LED_GPIO, led_state);
        ESP_LOGI(TAG, "LED %s", led_state ? "ON" : "OFF");
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

// Main
void app_main() {
    // LED Blink
    ESP_LOGI(TAG, "Blink LED Starting...");

    xTaskCreate(led_blink, "led_blink", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Blink LED Successfully Initialized.");
}