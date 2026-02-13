/**
 * @file main.cpp
 * @author Anthony Yalong
 * @brief Multi-sensor ADC data acquisition system using FreeRTOS queues for 
 *        producer-consumer pattern with two independent sensor tasks and 
 *        centralized aggregation/display
 */

// includes
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// project configuration
#define BAUDRATE 115200
#define ADC1_PIN GPIO_NUM_36
#define ADC2_PIN GPIO_NUM_39

// task configuration
#define SENSOR_TASK_STACK_SIZE 2048
#define SENSOR_1_TASK_DELAY_MS 500
#define SENSOR_2_TASK_DELAY_MS 750
#define AGGREGATOR_TASK_STACK_SIZE 4096

// adc configuration
#define ADC_ATTEN ADC_11db
#define ADC_BITWIDTH 12

// queue configuration
#define QUEUE_TIMEOUT_MS 100
#define QUEUE_SIZE 20

// sensor structure
typedef struct {
  gpio_num_t adc_pin;
  TickType_t delay_tick;
  TickType_t last_wake;
} sensor_config_t;

// display structure
typedef struct {
  gpio_num_t source;
  uint16_t raw_value;
  uint32_t time_stamp;
} sensor_message_t;

// globals
static const char *TAG = "queues_advanced (arduino)";
static QueueHandle_t event_queue = NULL;
static sensor_config_t sensor_1_config;
static sensor_config_t sensor_2_config;

// function prototypes
void sensor_task(void *pvParameters);
void aggregator_task(void *pvParameters);

void setup() {
  // serial
  Serial.begin(BAUDRATE);
  if (!Serial) delay(10);

  // error managment
  BaseType_t xRet;

  // adc initialization
  analogSetAttenuation(ADC_ATTEN);
  analogReadResolution(ADC_BITWIDTH);

  // queue initialization
  event_queue = xQueueCreate(QUEUE_SIZE, sizeof(sensor_message_t));
  if (event_queue == NULL) {
    Serial.printf("%s: failed to initialize event queue\n");
    while (true) { vTaskDelay(portMAX_DELAY); }
  }

  // sensor 1 task creation
  sensor_1_config.adc_pin = ADC1_PIN;
  sensor_1_config.delay_tick = pdMS_TO_TICKS(SENSOR_1_TASK_DELAY_MS);
  xRet = xTaskCreatePinnedToCore(
    sensor_task,
    "sensor_1_task",
    SENSOR_TASK_STACK_SIZE,
    &sensor_1_config,
    1,
    NULL,
    1
  );
  if (xRet != pdTRUE) {
    Serial.printf("%s: failed to create sensor 1 task\n");
    while (true) { vTaskDelay(portMAX_DELAY); }
  }

  // sensor 2 task creation
  sensor_2_config.adc_pin = ADC2_PIN;
  sensor_2_config.delay_tick = pdMS_TO_TICKS(SENSOR_2_TASK_DELAY_MS);
  xRet = xTaskCreatePinnedToCore(
    sensor_task,
    "sensor_2_task",
    SENSOR_TASK_STACK_SIZE,
    &sensor_2_config,
    1,
    NULL,
    1
  );
  if (xRet != pdTRUE) {
    Serial.printf("%s: failed to create sensor 2 task\n");
    while (true) { vTaskDelay(portMAX_DELAY); }
  }

  // aggregator task creation
  xRet = xTaskCreatePinnedToCore(
    aggregator_task,
    "aggregator_task",
    AGGREGATOR_TASK_STACK_SIZE,
    NULL,
    2,
    NULL,
    1
  );
  if (xRet != pdTRUE) {
    Serial.printf("%s: failed to create aggregator task\n");
    while (true) { vTaskDelay(portMAX_DELAY); }
  }
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void sensor_task(void *pvParameters) {
  // setup
  sensor_config_t *sensor_config = (sensor_config_t *)pvParameters;
  sensor_config->last_wake = xTaskGetTickCount();
  sensor_message_t message;

  // initialization
  message.source = sensor_config->adc_pin;

  while (true) {
    // read value
    message.raw_value = analogRead(sensor_config->adc_pin);

    // timestamp
    message.time_stamp = millis();

    // enqueue
    if (xQueueSend(event_queue, &message, pdMS_TO_TICKS(QUEUE_TIMEOUT_MS)) != pdPASS) {
      Serial.printf("%s: queue full! dropped sample! source: GPIO%d\n", TAG, sensor_config->adc_pin);
    }

    vTaskDelayUntil(&sensor_config->last_wake, sensor_config->delay_tick);
  }
}

void aggregator_task(void *pvParameters) {
  // setup
  sensor_message_t message;

  while (1) {
    // dequeue
    if (xQueueReceive(event_queue, &message, portMAX_DELAY) == pdPASS) {
      float voltage = (message.raw_value / 4095.0f) * 3.3f;
      Serial.printf("GPIO%d | T=%lu ms | Raw=%4d | %.2f V | Queue: %d/%d\n",
        message.source,
        message.time_stamp,
        message.raw_value,
        voltage, 
        uxQueueMessagesWaiting(event_queue),
        QUEUE_SIZE
      );
    }
  }
}