// Author: Anthony Yalong
// Description: ESP32 button debounce with LED toggle using FreeRTOS task and interrupts.
// Note: See TODOs

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <esp_timer.h>

// Hardware configuration
#define LED_NUM GPIO_NUM_2
#define SWITCH_NUM GPIO_NUM_4

// Debounce configuration
#define DEBOUNCE_TIME_MS 50

// Logging
static const char *TAG = "Debounce INTR (ESP-IDF)";

// Debounce global variables
static esp_timer_handle_t debounce_timer;
static volatile bool timer_active = false;
static portMUX_TYPE timer_spinlock = portMUX_INITIALIZER_UNLOCKED;

// LED handler
typedef struct
{
    gpio_num_t led_pin;
    bool led_state;
} led_handler_t;

// TODO: Improve error types
typedef enum
{
    DEBOUNCE_INTR_OK = 0,
    DEBOUNCE_INTR_INIT_ERR,
} debounce_err_t;
/**
 * @brief Initialize LED GPIO configuration and handler structure
 * @param self Pointer to LED handler structure to initialize
 * @param pin GPIO pin number for the LED
 * @return DEBOUNCE_INTR_OK on success, DEBOUNCE_INTR_INIT_ERR on failure
 */
static debounce_err_t gpio_led_init(led_handler_t *self, gpio_num_t pin);

/**
 * @brief Initialize switch GPIO configuration with interrupt capability
 * @param pin GPIO pin number for the switch/button
 * @return DEBOUNCE_INTR_OK on success, DEBOUNCE_INTR_INIT_ERR on failure
 */
static debounce_err_t gpio_switch_init(gpio_num_t pin);

/**
 * @brief Initialize both LED and switch GPIO configurations
 * @param self Pointer to LED handler structure to initialize
 * @param led_pin GPIO pin number for the LED
 * @param switch_pin GPIO pin number for the switch/button
 * @return DEBOUNCE_INTR_OK on success, DEBOUNCE_INTR_INIT_ERR on failure
 */
static debounce_err_t gpio_init(led_handler_t *self, gpio_num_t led_pin, gpio_num_t switch_pin);

/**
 * @brief GPIO interrupt service routine for button press detection
 * @param arg Unused parameter (required by ISR signature)
 * @note This function is placed in IRAM for fast execution
 * @note Starts/restarts debounce timer to filter switch bounce
 */
static void IRAM_ATTR gpio_isr_handler(void *arg);

/**
 * @brief Timer callback function executed after debounce period
 * @param arg Pointer to led_handler_t structure
 * @note Toggles LED state if button is still pressed after debounce delay
 */
static void debounce_timer_callback(void *arg);

/**
 * @brief Clean up allocated resources and GPIO configurations
 * @note Stops and deletes timer, removes ISR handlers, uninstalls ISR service
 */
static void app_cleanup(void);

/**
 * @brief Main application
 */
void app_main()
{
    debounce_err_t debounce_err_ret;
    esp_err_t esp_err_ret;
    led_handler_t led_handler;

    // Initialize GPIO
    debounce_err_ret = gpio_init(&led_handler, LED_NUM, SWITCH_NUM);
    if (debounce_err_ret != DEBOUNCE_INTR_OK)
    {
        ESP_LOGE(TAG, "app_main: failed to initialize gpio");
        goto error_exit;
    }

    // Install ISR
    esp_err_ret = gpio_install_isr_service(0);
    if (esp_err_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "app_main: failed to install ISR service");
        goto error_exit;
    }

    // Add ISR to switch
    esp_err_ret = gpio_isr_handler_add(SWITCH_NUM, gpio_isr_handler, NULL);
    if (esp_err_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "app_main: failed to add ISR to requested gpio pin");
        goto error_exit;
    }

    // Create ISR debounce timer
    esp_timer_create_args_t timer_args = {
        .arg = &led_handler,
        .callback = debounce_timer_callback,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "debounce_timer",
        .skip_unhandled_events = false,
    };
    esp_err_ret = esp_timer_create(&timer_args, &debounce_timer);
    if (esp_err_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "app_main: failed to create ISR debounce timer");
        goto error_exit;
    }

    // Keep main task alive
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

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

// TODO: Add a fallback case
static debounce_err_t gpio_led_init(led_handler_t *self, gpio_num_t pin)
{
    // Check GPIO pin validity
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin))
    {
        ESP_LOGE(TAG, "gpio_led_init: Invalid pin selection for LED");
        return DEBOUNCE_INTR_INIT_ERR;
    }

    // LED configuration
    gpio_config_t led_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    // Error handling initialization
    esp_err_t ret;

    // Set LED configuration
    ret = gpio_config(&led_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_led_init: Failed to configure LED GPIO");
        return DEBOUNCE_INTR_INIT_ERR;
    }

    // Set initial LED level
    ret = gpio_set_level(pin, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_led_init: Failed to set LED");
        return DEBOUNCE_INTR_INIT_ERR;
    }

    // Successful LED initalization
    self->led_pin = pin;
    self->led_state = false;
    ESP_LOGI(TAG, "gpio_led_init: LED initialized successfully");
    return DEBOUNCE_INTR_OK;
}

// TODO: Add a Fallback case
static debounce_err_t gpio_switch_init(gpio_num_t pin)
{
    // Check GPIO pin validity
    if (!GPIO_IS_VALID_GPIO(pin))
    {
        ESP_LOGE(TAG, "gpio_switch_init: Invalid pin selection for switch");
        return DEBOUNCE_INTR_INIT_ERR;
    }

    // Switch configuration
    gpio_config_t switch_config = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    // Error handling initialization
    esp_err_t ret;

    // Set switch configuration
    ret = gpio_config(&switch_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_switch_init: Failed to configure switch GPIO");
        return DEBOUNCE_INTR_INIT_ERR;
    }

    // Successful switch initalization
    ESP_LOGI(TAG, "gpio_switch_init: switch initialized successfully");
    return DEBOUNCE_INTR_OK;
}

static debounce_err_t gpio_init(led_handler_t *self, gpio_num_t led_pin, gpio_num_t switch_pin)
{
    // Initialize LED and switch
    debounce_err_t gpio_led_init_ret = gpio_led_init(self, led_pin);
    debounce_err_t gpio_switch_init_ret = gpio_switch_init(switch_pin);

    if (gpio_led_init_ret != DEBOUNCE_INTR_OK || gpio_switch_init_ret != DEBOUNCE_INTR_OK)
    {
        return DEBOUNCE_INTR_INIT_ERR;
    }

    return DEBOUNCE_INTR_OK;
}

// TODO: Add error checking
static void gpio_isr_handler(void *arg)
{
    // start/restart debounce timer
    portENTER_CRITICAL_ISR(&timer_spinlock);
    if (!timer_active)
    {
        esp_timer_start_once(debounce_timer, DEBOUNCE_TIME_MS * 1000);
        timer_active = true;
    }
    else
    {
        esp_timer_stop(debounce_timer);
        esp_timer_start_once(debounce_timer, DEBOUNCE_TIME_MS * 1000);
    }
    portEXIT_CRITICAL_ISR(&timer_spinlock);
}

// TODO: Add error checking
static void debounce_timer_callback(void *arg)
{
    led_handler_t *led_handler = (led_handler_t *)arg;
    bool button_pressed = !gpio_get_level(SWITCH_NUM);
    portENTER_CRITICAL(&timer_spinlock);
    timer_active = false;
    portEXIT_CRITICAL(&timer_spinlock);

    // Toggle LED on rising-edge
    if (button_pressed)
    {
        led_handler->led_state = !(led_handler->led_state);
        gpio_set_level(led_handler->led_pin, led_handler->led_state);
        ESP_LOGI(TAG, "LED %s", led_handler->led_state ? "ON" : "OFF");
    }
}

static void app_cleanup(void)
{
    // Clean timer
    if (debounce_timer)
    {
        esp_timer_stop(debounce_timer);
        esp_timer_delete(debounce_timer);
        debounce_timer = NULL;
    }

    // Clean ISR
    if (gpio_isr_handler_remove(SWITCH_NUM) == ESP_OK)
    {
        gpio_uninstall_isr_service();
    }

    ESP_LOGI(TAG, "Application cleanup completed");
}