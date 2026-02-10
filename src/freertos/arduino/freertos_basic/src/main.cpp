/**
 * @file main.cpp
 * @author Anthony Yalong
 * @brief Dual LED blink using FreeRTOS tasks demonstrating independent task execution using Arduino framework.
 */

// imports
#include <Arduino.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// project configuration
#define BAUDRATE 115200
#define TASK_STACK_DEPTH 4096
#define TASK_PRIORITY 5

// led1 task configuration
#define LED1_TASK_PIN GPIO_NUM_22
#define LED1_TASK_DELAY_MS 500

// led2 task configuration
#define LED2_TASK_PIN GPIO_NUM_23
#define LED2_TASK_DELAY_MS 250

// logging
const char *TAG = "freertos_basic (arduino)";

// led task structure
typedef struct {
  gpio_num_t pin;
  bool state;
  TickType_t delay_ticks;
  TickType_t last_wake;
} led_task_t;

// task configuration
static TaskHandle_t led1_task_handle;
static TaskHandle_t led2_task_handle;
static led_task_t led1_task;
static led_task_t led2_task;

// function prototypes
static void led_task(void *pvParameters);

void setup() {
  // initialize serial
  Serial.begin(BAUDRATE);
  while (!Serial) {
    delay(10);
  }

  // initialize task1
  led1_task = {
    .pin = LED1_TASK_PIN,
    .state = LOW,
    .delay_ticks = pdMS_TO_TICKS(LED1_TASK_DELAY_MS)
  };
  pinMode(LED1_TASK_PIN, OUTPUT);
  digitalWrite(LED1_TASK_PIN, LOW);
  
  xTaskCreate(
    led_task,
    "led1_task",
    TASK_STACK_DEPTH,
    &led1_task,
    TASK_PRIORITY,
    &led1_task_handle
  );

  // initialize task2
  led2_task = {
    .pin = LED2_TASK_PIN,
    .state = LOW,
    .delay_ticks = pdMS_TO_TICKS(LED2_TASK_DELAY_MS)
  };
  pinMode(LED2_TASK_PIN, OUTPUT);
  digitalWrite(LED2_TASK_PIN, LOW);

  xTaskCreate(
    led_task,
    "led2_task",
    TASK_STACK_DEPTH,
    &led2_task,
    TASK_PRIORITY,
    &led2_task_handle
  );
}

void loop() {
  vTaskDelay(5000 / portTICK_PERIOD_MS);
    
  UBaseType_t stack1 = uxTaskGetStackHighWaterMark(led1_task_handle);
  UBaseType_t stack2 = uxTaskGetStackHighWaterMark(led2_task_handle);
  
  Serial.printf("%s: task1 free stack: %u bytes\n", TAG, stack1 * 4);
  Serial.printf("%s: task2 free stack: %u bytes\n", TAG, stack2 * 4);
}

static void led_task(void *pvParameters) {
  // read parameter
  led_task_t *config = (led_task_t *) pvParameters;

  // initalize timing
  config->last_wake = xTaskGetTickCount();

  while (1) {
    // update state
    config->state = !config->state;

    // toggle led 
    digitalWrite(config->pin, config->state);
    Serial.printf("%s: led %s\n", TAG, config->state ? "on" : "off");

    // timing
    vTaskDelayUntil(&config->last_wake, config->delay_ticks);
  }

  // safety
  vTaskDelete(NULL);
}