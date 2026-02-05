// Author: Anthony Yalong
// Description: ESP32 button debounce with LED toggle using FreeRTOS task and dual-stage filtering (sample count + time delay)

#include <esp_system.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// GPIO config
#define LED_NUM GPIO_NUM_2
#define SWITCH_NUM GPIO_NUM_4

// Logging
static const char *TAG = "Debounce (ESP-IDF)";

// LED variable
bool led_state = false;

// Debounce struct
typedef struct
{
    int pin;
    bool inverted;

    // States
    bool cur_state;
    bool prev_stable_state;

    // Sampling
    uint8_t consecutive_samples;
    uint8_t required_samples;

    // Timing
    TickType_t last_change_tick;
    TickType_t delay_tick;
} Software_Debounce;
Software_Debounce debouncer;

typedef enum
{
    DEBOUNCE_NO_EDGE,
    DEBOUNCE_RISING,
    DEBOUNCE_FALLING
} debounce_edge_t;

// Function definitions
void gpio_init(void);
bool debounce_read(gpio_num_t pin_num, bool inverted);
void debounce_init(Software_Debounce *self, gpio_num_t pin_num, bool debounce_inverted, uint32_t debounce_delay_ms, uint8_t debounce_required_samples);
debounce_edge_t debounce_update(Software_Debounce *self);
void debounce_task(void *pvParameters);

void app_main()
{
    ESP_LOGI(TAG, "Starting program.");

    // Initialize
    gpio_init();
    debounce_init(&debouncer, SWITCH_NUM, true, 50, 3);

    // Create Task
    xTaskCreate(debounce_task, "Switch Debounce", 2048, NULL, 5, NULL);
}

void gpio_init(void)
{
    // LED config
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_config);
    gpio_set_level(LED_NUM, led_state);

    // Switch config
    gpio_config_t switch_config = {
        .pin_bit_mask = (1ULL << SWITCH_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&switch_config);
}

bool debounce_read(gpio_num_t pin_num, bool inverted)
{
    return inverted ? !gpio_get_level(pin_num) : gpio_get_level(pin_num);
}

void debounce_init(Software_Debounce *self, gpio_num_t pin_num, bool debounce_inverted, uint32_t debounce_delay_ms, uint8_t debounce_required_samples)
{
    // Null pointer check
    if (!self)
        return;

    self->pin = pin_num;
    self->inverted = debounce_inverted;
    self->delay_tick = pdMS_TO_TICKS(debounce_delay_ms);

    // Initialize stable state
    bool initial_state = debounce_read(self->pin, self->inverted);
    self->cur_state = initial_state;
    self->prev_stable_state = initial_state;

    // Sampling
    self->required_samples = debounce_required_samples;
    self->consecutive_samples = 0;

    // Timing
    self->last_change_tick = xTaskGetTickCount();
}

debounce_edge_t debounce_update(Software_Debounce *self)
{
    bool raw_state = debounce_read(self->pin, self->inverted);
    TickType_t now = xTaskGetTickCount();

    // Reset if state changed
    if (raw_state != self->cur_state)
    {
        self->cur_state = raw_state;
        self->consecutive_samples = 0;
        self->last_change_tick = now;
        return DEBOUNCE_NO_EDGE;
    }

    // Need more samples
    if (self->consecutive_samples < self->required_samples)
    {
        self->consecutive_samples++;
        return DEBOUNCE_NO_EDGE;
    }

    // Need more time
    if ((now - self->last_change_tick) < self->delay_tick)
    {
        return DEBOUNCE_NO_EDGE;
    }

    // Check if actual state changed
    if (raw_state != self->prev_stable_state)
    {
        self->prev_stable_state = raw_state;
        return raw_state ? DEBOUNCE_RISING : DEBOUNCE_FALLING;
    }

    return DEBOUNCE_NO_EDGE;
}

void debounce_task(void *pvParameters)
{
    while (1)
    {
        debounce_edge_t edge = debounce_update(&debouncer);

        // Update on rising edge
        if (edge == DEBOUNCE_RISING)
        {
            led_state = !led_state;
            gpio_set_level(LED_NUM, led_state);
            ESP_LOGI(TAG, "%s", led_state ? "LED ON" : "LED OFF");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
