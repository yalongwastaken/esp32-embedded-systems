/**
 * @file main.cpp
 * @author Anthony Yalong
 * @brief FreeRTOS queue demo using a producer-consumer pattern to sample ADC readings and print converted voltages. Arduino framework. 
 */

// imports
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

// project configuration
#define BAUDRATE 115200

// adc configuration
#define ADC_PIN GPIO_NUM_36
#define ADC_ATTEN ADC_11db    // 0 - 3.3V
#define ADC_RESOLUTION 12     // 12 bit resolution

// queue configuration
#define QUEUE_DEPTH 10
#define QUEUE_TIMEOUT_MS 100

// task configuration
#define ADC_TASK_DELAY_MS 500

static QueueHandle_t event_queue = NULL;
static const char *TAG = "queue_basic (arduino)";

// function prototypes
static void adc_task(void *pvParameters);
static void print_task(void *pvParameters);

void setup() {
  // error management
  BaseType_t xRet;

  Serial.begin(BAUDRATE);
  if (!Serial) delay(10);

  // initialize adc
  analogSetAttenuation(ADC_ATTEN);
  analogReadResolution(ADC_RESOLUTION);


  // create queue
  event_queue = xQueueCreate(QUEUE_DEPTH, sizeof(uint16_t));
  if (event_queue == NULL) {
    Serial.printf("%s: failed to initialize queue\n", TAG);
    while (true) { vTaskDelay(portMAX_DELAY); }
  }

  // create adc task
  xRet = xTaskCreatePinnedToCore(
    adc_task,
    "adc_task",
    2048,
    NULL,
    1,
    NULL,
    1
  );
  if (xRet != pdTRUE) {
    Serial.printf("%s: failed to create adc task\n", TAG);
    while (true) { vTaskDelay(portMAX_DELAY); }
  }

  // create print task
  xRet = xTaskCreatePinnedToCore(
    print_task,
    "print_task",
    2048,
    NULL,
    1,
    NULL,
    1
  );
  if (xRet != pdTRUE) {
    Serial.printf("%s: failed to create print task\n", TAG);
    while (true) { vTaskDelay(portMAX_DELAY); }
  }
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

static void adc_task(void *pvParameters) {
  uint16_t raw_value;
  TickType_t adc_last_wake = xTaskGetTickCount();

  while (true) {
    raw_value = analogRead(ADC_PIN);

    // enqueue
    if (xQueueSend(event_queue, &raw_value, pdMS_TO_TICKS(QUEUE_TIMEOUT_MS)) != pdPASS) {
      Serial.printf("%s: queue full! sample dropped!\n", TAG);
    }

    vTaskDelayUntil(&adc_last_wake, pdMS_TO_TICKS(ADC_TASK_DELAY_MS));
  }
}

static void print_task(void *pvParameters) {
  uint16_t received;
  while (true) {
    if (xQueueReceive(event_queue, &received, portMAX_DELAY) == pdPASS) {
      // convert to voltage range
      float voltage = (received / 4095.0f) * 3.3f;

      // print
      Serial.printf("%s: voltage: %0.1f\n", TAG, voltage);
    }
  }
}