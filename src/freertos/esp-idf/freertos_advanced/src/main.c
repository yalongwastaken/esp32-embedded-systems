/**
 * @file main.c
 * @author Anthony Yalong
 * @brief
 * Demonstrates FreeRTOS priority-based scheduling on the ESP32 using the
 * esp-idf framework, with three tasks and an ISR-triggered high-priority task.
 *
 * Shows task preemption, ISR-to-task notification, and starvation of
 * lower-priority tasks under sustained high-priority activity.
 */

// includes
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// task configuration
#define TASK_STACK_DEPTH    2048
#define LED_TASK_HIGH_PIN   GPIO_NUM_12
#define LED_TASK_MED_PIN    GPIO_NUM_13
#define LED_TASK_LOW_PIN    GPIO_NUM_14
#define SWITCH_PIN          GPIO_NUM_15

// logging
static const char *TAG = "freertos_advanced (esp-idf)";

// task handles 
static TaskHandle_t high_prio_task_handle;
static TaskHandle_t med_prio_task_handle;
static TaskHandle_t low_prio_task_handle;

// volatile variables
volatile bool high_prio_active;

// function prototypes
static void switch_isr(void *arg);
static esp_err_t leds_init(void);
static esp_err_t switch_init(void);
static void high_prio_task(void *pvParameters);
static void med_prio_task(void *pvParameters);
static void low_prio_task(void *pvParameters);

void app_main() {
    // error managment
    esp_err_t ret;
    BaseType_t xRet;

    // initialize led gpios
    ret = leds_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize leds");
        return;
    }

    // initialize switch
    ret = switch_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize switch");
        return;
    }

    // initialize isr
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to install isr service");
        return;
    }
    ret = gpio_isr_handler_add(SWITCH_PIN, switch_isr, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to add gpio isr handler");
        return;
    }

    // create high prio task
    xRet = xTaskCreatePinnedToCore(
        high_prio_task,
        "high_prio_task",
        TASK_STACK_DEPTH,
        NULL,
        3,
        &high_prio_task_handle,
        1
    );
    if (xRet != pdTRUE) {
        ESP_LOGE(TAG, "failed to create high priority task");
        return;
    }

    // create med prior task
    xRet = xTaskCreatePinnedToCore(
        med_prio_task,
        "med_prio_task",
        TASK_STACK_DEPTH,
        NULL,
        2,
        &med_prio_task_handle,
        1
    );
    if (xRet != pdTRUE) {
        ESP_LOGE(TAG, "failed to create med priority task");
        return;
    }

    // create low prior task
    xRet = xTaskCreatePinnedToCore(
        low_prio_task,
        "low_prio_task",
        TASK_STACK_DEPTH,
        NULL,
        1,
        &low_prio_task_handle,
        1
    );
    if (xRet != pdTRUE) {
        ESP_LOGE(TAG, "failed to create low priority task");
        return;
    }

    while (true) {
        // monitor stack usage every 10 seconds
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "--- stack usage ---");
        ESP_LOGI(TAG, "high priority: %lu bytes free", (unsigned long) uxTaskGetStackHighWaterMark(high_prio_task_handle) * 4);
        ESP_LOGI(TAG, "medium priority: %lu bytes free", (unsigned long) uxTaskGetStackHighWaterMark(med_prio_task_handle) * 4);
        ESP_LOGI(TAG, "low priority: %lu bytes free", (unsigned long) uxTaskGetStackHighWaterMark(low_prio_task_handle) * 4);
    }
}

static void IRAM_ATTR switch_isr(void *arg) {
    high_prio_active = !high_prio_active;

    // notify high priority task from isr
    BaseType_t higher_prio_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(high_prio_task_handle, &higher_prio_task_woken);

    // yield 
    if (higher_prio_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t leds_init(void) {
    // error management
    esp_err_t ret;

    // configuration
    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << LED_TASK_HIGH_PIN) | (1ULL << LED_TASK_MED_PIN) | (1ULL << LED_TASK_LOW_PIN)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize led gpio pins");
        return ret;
    }

    // initial states
    gpio_set_level(LED_TASK_HIGH_PIN, false);
    gpio_set_level(LED_TASK_MED_PIN, false);
    gpio_set_level(LED_TASK_LOW_PIN, false);

    ESP_LOGI(TAG, "successfully initialized led gpio pins");
    return ESP_OK;
}

static esp_err_t switch_init(void) {
    // error management
    esp_err_t ret;

    // configuration
    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SWITCH_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ret = gpio_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize switch gpio");
        return ret;
    }

    ESP_LOGI(TAG, "successfully initialized switch gpio pin");
    return ESP_OK;
}

static void high_prio_task(void *pvParameters) {
    while (true) {
        // wait for notification from switch isr
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (high_prio_active) {
            ESP_LOGI(TAG, "HIGH PRIORITY TASK ACTIVE");

            // blink
            for (int i = 0; i < 50; i++) {
                gpio_set_level(LED_TASK_HIGH_PIN, true);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(LED_TASK_HIGH_PIN, false);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }

            ESP_LOGI(TAG, "HIGH PRIORITY TASK COMPLETE");
        }
    }
}

static void med_prio_task(void *pvParameters) {
    uint32_t loop_count = 0;
        
    while (true) {
        gpio_set_level(LED_TASK_MED_PIN, true);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        gpio_set_level(LED_TASK_MED_PIN, false);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        
        // log
        if (++loop_count % 10 == 0) {
            ESP_LOGI(TAG, "medium priority task running (count: %lu)", loop_count);
        }
    }
}

static void low_prio_task(void *pvParameters) {
    uint32_t loop_count = 0;
        
    while (true) {
        gpio_set_level(LED_TASK_LOW_PIN, true);
        vTaskDelay(700 / portTICK_PERIOD_MS);
        gpio_set_level(LED_TASK_LOW_PIN, false);
        vTaskDelay(700 / portTICK_PERIOD_MS);
        
        // log
        if (++loop_count % 10 == 0) {
            ESP_LOGI(TAG, "low priority task running (count: %lu)", loop_count);
        }
    }
}