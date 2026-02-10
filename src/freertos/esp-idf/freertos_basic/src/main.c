/**
 * @file main.c
 * @author Anthony Yalong
 * @brief Dual LED blink using FreeRTOS tasks demonstrating independent task execution using esp-idf framework.
 */

// includes
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// task configuration
#define TASK_STACK_DEPTH 4096
#define TASK_PRIORITY 5
#define LOG_DELAY 10000

// led1 task configuration
#define LED1_TASK_PIN GPIO_NUM_22
#define LED1_TASK_DELAY_MS 500

// led2 task configuration
#define LED2_TASK_PIN GPIO_NUM_23
#define LED2_TASK_DELAY_MS 250

// logging
static const char *TAG = "freertos_basic (esp-idf)";

// led task structure
typedef struct {
  gpio_num_t pin;
  bool state;
  TickType_t delay_ticks;
  TickType_t last_wake;
} led_struct_t;

// task configuration
static TaskHandle_t led1_task_handle;
static TaskHandle_t led2_task_handle;
static led_struct_t led1_task_struct;
static led_struct_t led2_task_struct;

// function prototypes
static esp_err_t led_init(led_struct_t *led_struct, gpio_num_t pin, uint32_t delay_ms);
static void led_task(void *pvParameters);

void app_main() {
    // error management
    esp_err_t ret;
    BaseType_t xRet;

    // initialize led1 
    ret = led_init(&led1_task_struct, LED1_TASK_PIN, LED1_TASK_DELAY_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize led1 task gpio");
        return;
    }

    // initialize led2
    ret = led_init(&led2_task_struct, LED2_TASK_PIN, LED2_TASK_DELAY_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize led2 task gpio");
        return;
    }

    // led1 task create
    xRet = xTaskCreate(
        led_task,
        "led_task1",
        TASK_STACK_DEPTH,
        &led1_task_struct,
        TASK_PRIORITY,
        &led1_task_handle 
    );
    if (xRet != pdPASS) {
        ESP_LOGE(TAG, "failed to create led1 task");
        return;
    }

    // led2 task create
    xRet = xTaskCreate(
        led_task,
        "led_task2",
        TASK_STACK_DEPTH,
        &led2_task_struct,
        TASK_PRIORITY,
        &led2_task_handle 
    );
    if (xRet != pdPASS) {
        ESP_LOGE(TAG, "failed to create led2 task");
        return;
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(LOG_DELAY));
        
        // log free heap
        ESP_LOGI(TAG, "free heap: %lu bytes", esp_get_free_heap_size());
    }
}

static esp_err_t led_init(led_struct_t *led_struct, gpio_num_t pin, uint32_t delay_ms) {
    // error managment
    esp_err_t ret;

    // gpio config
    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << pin),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ret = gpio_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure gpio pin on GPIO%d", pin);
        return ret;
    }

    // initial state
    gpio_set_level(pin, false);

    // initialize led struct
    led_struct->pin = pin;
    led_struct->delay_ticks = pdMS_TO_TICKS(delay_ms);
    led_struct->state = false;

    ESP_LOGI(TAG, "successfully initialized led on gpio pin: GPIO%d", pin);
    return ESP_OK;
}

static void led_task(void *pvParameters) {
    led_struct_t *config = (led_struct_t *)pvParameters;
    ESP_LOGI(TAG, "task %s: GPIO%d started on core %d", pcTaskGetName(NULL), config->pin, xPortGetCoreID());


    // setup
    config->last_wake = xTaskGetTickCount();

    while (true) {
        // update state
        config->state = !config->state;

        // update led
        gpio_set_level(config->pin, config->state);

        // delay
        vTaskDelayUntil(&config->last_wake, config->delay_ticks);
    }

    // safety
    vTaskDelete(NULL);
}