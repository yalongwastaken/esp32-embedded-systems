// Author: Anthony Yalong
// Description: Bare-bones switch-LED integration using ESP32 (ESP-IDF framework). Reads a push-button input and controls an LED output via GPIO

#include <esp_system.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LED_NUM GPIO_NUM_2
#define SWITCH_NUM GPIO_NUM_4

// variables
static const char *TAG = "Switch (ESP-IDF)";
bool is_pressed;
bool prev_is_pressed;

// Function definition
void init_gpio(void);
void switch_control(void *pvParameters);

void app_main() {
    init_gpio();

    xTaskCreate(switch_control, "Switch Control", 2048, NULL, 1, NULL);
}

void init_gpio(void) {
    // LED config
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_config);
    gpio_set_level(LED_NUM, 0);

    // Switch config
    gpio_config_t switch_config = {
        .pin_bit_mask = (1ULL << SWITCH_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&switch_config);
    is_pressed = !gpio_get_level(SWITCH_NUM);
    prev_is_pressed = is_pressed;
}

void switch_control(void *pvParameters) {
    while (1) {
        is_pressed = !gpio_get_level(SWITCH_NUM);
        gpio_set_level(LED_NUM, is_pressed ? 1 : 0);
        if (is_pressed != prev_is_pressed) {
            ESP_LOGI(TAG, "%s", is_pressed ? "LED ON" : "LED OFF");
            prev_is_pressed = is_pressed;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}