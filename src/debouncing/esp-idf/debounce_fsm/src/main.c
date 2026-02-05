// Author: Anthony Yalong
// Description: ESP-IDF FSM-based button debouncer with hysteresis for reliable rising-edge detection.

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string.h>

// Hardware Configuration
#define LED_NUM GPIO_NUM_2
#define SWITCH_NUM GPIO_NUM_4
#define TASK_POLL_RATE_MS 10
#define TASK_STACK_SIZE 2048

// Configuration constants
#define DEFAULT_PRESS_DELAY_MS 50
#define DEFAULT_RELEASE_DELAY_MS 20
#define MAX_DELAY_MS 1000

// Logging
static const char *TAG = "Debounce FSM (ESP-IDF)";

// Error codes
typedef enum
{
    DEBOUNCE_OK = 0,
    DEBOUNCE_ERR_NULL_POINTER,
    DEBOUNCE_ERR_INVALID_GPIO,
    DEBOUNCE_ERR_INVALID_DELAY,
    DEBOUNCE_ERR_INIT_FAILED,
    DEBOUNCE_ERR_HARDWARE_FAULT
} debounce_error_t;

// Debounce state definitions
typedef enum
{
    DEBOUNCE_LOW,
    DEBOUNCE_RISING,
    DEBOUNCE_HIGH,
    DEBOUNCE_FALLING,
} debounce_state_t;

// Debounce edge tracking
typedef enum
{
    DEBOUNCE_NO_EDGE,
    DEBOUNCE_RISING_EDGE,
    DEBOUNCE_FALLING_EDGE,
} debounce_edge_t;

// Button event callback type
typedef void (*button_event_callback_t)(debounce_edge_t edge, void *device_info);

// Debounce
typedef struct
{
    // Hardware config
    gpio_num_t pin;
    bool inverted;

    // Timing config
    TickType_t press_delay_ticks;
    TickType_t release_delay_ticks;

    // State config
    debounce_state_t state;
    TickType_t state_change_time;

    // Callback config
    button_event_callback_t callback;
    void *device_info;

    // Error config
    bool initialized;
} debounce_t;

// LED controller
typedef struct
{
    gpio_num_t pin;
    bool state;
    bool initialized;
} led_controller_t;

// Global instances
static debounce_t debouncer;
static led_controller_t led_controller;

// Task handle
static TaskHandle_t debounce_task_handle = NULL;

/**
 * @brief Read GPIO pin with proper error handling
 * @param pin GPIO pin number to read
 * @param inverted true to invert the reading
 * @param result Pointer to store the logical pin state
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t gpio_read_pin(gpio_num_t pin, bool inverted, bool *result)
{
    if (!result)
    {
        ESP_LOGE(TAG, "gpio_read_pin: result pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!GPIO_IS_VALID_GPIO(pin))
    {
        ESP_LOGE(TAG, "gpio_read_pin: Invalid GPIO pin %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    int raw_level = gpio_get_level(pin);
    if (raw_level < 0)
    {
        ESP_LOGE(TAG, "gpio_read_pin: Failed to read GPIO %d", pin);
        return ESP_FAIL;
    }

    *result = inverted ? !raw_level : raw_level;
    return ESP_OK;
}

/**
 * @brief Setup LED GPIO
 */
static esp_err_t gpio_setup_led(gpio_num_t pin)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin))
    {
        ESP_LOGE(TAG, "gpio_setup_led: Invalid output GPIO pin %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t led_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    esp_err_t ret = gpio_config(&led_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_setup_led: Configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize
    ret = gpio_set_level(pin, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_setup_led: Failed to set initial level: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "LED GPIO %d configured successfully", pin);
    return ESP_OK;
}

/**
 * @brief Setup button GPIO
 */
static esp_err_t gpio_setup_button(gpio_num_t pin)
{
    if (!GPIO_IS_VALID_GPIO(pin))
    {
        ESP_LOGE(TAG, "gpio_setup_button: Invalid GPIO pin %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t switch_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    esp_err_t ret = gpio_config(&switch_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_setup_button: Configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Button GPIO %d configured successfully", pin);
    return ESP_OK;
}

/**
 * @brief Initialize debouncer
 */
static debounce_error_t debounce_init(debounce_t *self, gpio_num_t pin, bool inverted,
                                      uint32_t press_delay_ms, uint32_t release_delay_ms,
                                      button_event_callback_t callback, void *device_info)
{
    // Input validation
    if (!self)
    {
        ESP_LOGE(TAG, "debounce_init: self pointer is NULL");
        return DEBOUNCE_ERR_NULL_POINTER;
    }

    if (!GPIO_IS_VALID_GPIO(pin))
    {
        ESP_LOGE(TAG, "debounce_init: Invalid GPIO pin %d", pin);
        return DEBOUNCE_ERR_INVALID_GPIO;
    }

    if (press_delay_ms == 0 || release_delay_ms == 0 ||
        press_delay_ms > MAX_DELAY_MS || release_delay_ms > MAX_DELAY_MS)
    {
        ESP_LOGE(TAG, "debounce_init: Invalid delay values (press=%lu, release=%lu)",
                 press_delay_ms, release_delay_ms);
        return DEBOUNCE_ERR_INVALID_DELAY;
    }

    // Clear structure
    memset(self, 0, sizeof(debounce_t));

    // Setup configuration
    self->pin = pin;
    self->inverted = inverted;
    self->press_delay_ticks = pdMS_TO_TICKS(press_delay_ms);
    self->release_delay_ticks = pdMS_TO_TICKS(release_delay_ms);
    self->callback = callback;
    self->device_info = device_info;

    // Initialize state
    bool current_state;
    esp_err_t read_ret = gpio_read_pin(pin, inverted, &current_state);
    if (read_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "debounce_init: Failed to read initial pin state");
        return DEBOUNCE_ERR_HARDWARE_FAULT;
    }

    self->state = current_state ? DEBOUNCE_HIGH : DEBOUNCE_LOW;
    self->state_change_time = xTaskGetTickCount();
    self->initialized = true;

    ESP_LOGI(TAG, "Debouncer initialized: pin=%d, inverted=%s, initial_state=%s",
             pin, inverted ? "true" : "false", current_state ? "HIGH" : "LOW");

    return DEBOUNCE_OK;
}

/**
 * @brief Update debouncer FSM
 */
static debounce_edge_t debounce_update(debounce_t *self)
{
    if (!self || !self->initialized)
    {
        ESP_LOGE(TAG, "debounce_update: Invalid debouncer instance");
        return DEBOUNCE_NO_EDGE;
    }

    bool current_pin_state;
    esp_err_t read_ret = gpio_read_pin(self->pin, self->inverted, &current_pin_state);
    if (read_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "debounce_update: Pin read error");
        return DEBOUNCE_NO_EDGE;
    }

    TickType_t now = xTaskGetTickCount();
    debounce_edge_t edge = DEBOUNCE_NO_EDGE;

    switch (self->state)
    {
    case DEBOUNCE_LOW:
        if (current_pin_state)
        {
            self->state = DEBOUNCE_RISING;
            self->state_change_time = now;
        }
        break;

    case DEBOUNCE_RISING:
        if (current_pin_state)
        {
            if ((now - self->state_change_time) >= self->press_delay_ticks)
            {
                self->state = DEBOUNCE_HIGH;
                edge = DEBOUNCE_RISING_EDGE;
            }
        }
        else
        {
            self->state = DEBOUNCE_LOW;
        }
        break;

    case DEBOUNCE_HIGH:
        if (!current_pin_state)
        {
            self->state = DEBOUNCE_FALLING;
            self->state_change_time = now;
        }
        break;

    case DEBOUNCE_FALLING:
        if (!current_pin_state)
        {
            if ((now - self->state_change_time) >= self->release_delay_ticks)
            {
                self->state = DEBOUNCE_LOW;
                edge = DEBOUNCE_FALLING_EDGE;
            }
        }
        else
        {
            self->state = DEBOUNCE_HIGH;
        }
        break;

    default:
        ESP_LOGE(TAG, "Invalid debouncer state: %d", self->state);
        return DEBOUNCE_NO_EDGE;
    }

    return edge;
}

/**
 * @brief Initialize LED controller
 */
static esp_err_t led_controller_init(led_controller_t *led, gpio_num_t pin)
{
    if (!led)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = gpio_setup_led(pin);
    if (ret != ESP_OK)
    {
        return ret;
    }

    led->pin = pin;
    led->state = false;
    led->initialized = true;

    return ESP_OK;
}

/**
 * @brief Set LED state
 */
static esp_err_t led_controller_set_state(led_controller_t *led, bool state)
{
    if (!led || !led->initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = gpio_set_level(led->pin, state ? 1 : 0);
    if (ret == ESP_OK)
    {
        led->state = state;
    }

    return ret;
}

/**
 * @brief Button event handler
 */
static void button_event_handler(debounce_edge_t edge, void *device_info)
{
    led_controller_t *led = (led_controller_t *)device_info;

    if (edge == DEBOUNCE_RISING_EDGE)
    {
        bool new_state = !led->state;
        esp_err_t ret = led_controller_set_state(led, new_state);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "LED %s", new_state ? "ON" : "OFF");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to set LED state: %s", esp_err_to_name(ret));
        }
    }
}

/**
 * @brief Main debounce task
 */
static void debounce_task(void *pvParameters)
{
    debounce_t *debouncer = (debounce_t *)pvParameters;

    if (!debouncer)
    {
        ESP_LOGE(TAG, "debounce_task: Invalid debouncer parameter");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Debounce task started");
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        debounce_edge_t edge = debounce_update(debouncer);

        if (edge != DEBOUNCE_NO_EDGE && debouncer->callback)
        {
            debouncer->callback(edge, debouncer->device_info);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_POLL_RATE_MS));
    }
}

/**
 * @brief Application cleanup
 */
static void app_cleanup(void)
{
    if (debounce_task_handle)
    {
        vTaskDelete(debounce_task_handle);
        debounce_task_handle = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Application cleanup completed");
}

/**
 * @brief Main application
 */
void app_main(void)
{
    esp_err_t ret;
    debounce_error_t debounce_ret;

    ESP_LOGI(TAG, "Application starting...");

    // Initialize LED controller
    ret = led_controller_init(&led_controller, LED_NUM);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LED controller: %s", esp_err_to_name(ret));
        goto error_exit;
    }

    // Setup button GPIO
    ret = gpio_setup_button(SWITCH_NUM);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to setup button GPIO: %s", esp_err_to_name(ret));
        goto error_exit;
    }

    // Initialize debouncer
    debounce_ret = debounce_init(&debouncer, SWITCH_NUM, true,
                                 DEFAULT_PRESS_DELAY_MS, DEFAULT_RELEASE_DELAY_MS,
                                 button_event_handler, &led_controller);
    if (debounce_ret != DEBOUNCE_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize debouncer: error code %d", debounce_ret);
        goto error_exit;
    }

    // Create debounce task
    BaseType_t task_ret = xTaskCreate(debounce_task, "debounce_task",
                                      TASK_STACK_SIZE, &debouncer,
                                      tskIDLE_PRIORITY + 1, &debounce_task_handle);
    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create debounce task");
        goto error_exit;
    }

    ESP_LOGI(TAG, "Application initialized successfully");
    return;

error_exit:
    ESP_LOGE(TAG, "Application failed to initialize - cleaning up");
    app_cleanup();

    // Prevent failure cycle
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}