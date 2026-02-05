// Author: Anthony Yalong
// Description: LED toggle control with software debounced button input.
//              Implements stable state detection to eliminate mechanical switch bounce.
//              LED toggles on each confirmed button press (rising edge detection).

#include <Arduino.h>

// Pin definitions
#define LED_NUM 2
#define SWITCH_NUM 4

// Debounce activators
typedef enum
{
  DEBOUNCE_NO_EDGE,
  DEBOUNCE_RISING_EDGE,
  DEBOUNCE_FALLING_EDGE,
} debounce_edge_t;

// Software debounce class
class SoftwareDebounce
{
private:
  // Class variables
  uint8_t pin;
  bool inverted;

  // States
  bool cur_state;
  bool prev_stable_state;

  // Timing
  unsigned long delay_time;
  unsigned long last_change_time;

  // Sampling
  unsigned int required_samples;
  unsigned int consecutive_samples;

public:
  // Constructor
  SoftwareDebounce(
      uint8_t switch_pin = SWITCH_NUM,
      bool inverted_logic = true,
      unsigned long debounce_delay_time = 50,
      unsigned int debounce_required_samples = 3)
  {
    pin = switch_pin;
    inverted = inverted_logic;

    // States
    pinMode(pin, inverted ? INPUT_PULLUP : INPUT_PULLDOWN);
    cur_state = prev_stable_state = read_raw_state(pin);

    // Timing
    delay_time = debounce_delay_time;
    last_change_time = 0;

    // Sampling
    required_samples = debounce_required_samples;
    consecutive_samples = 0;
  }

  // Update debounce
  debounce_edge_t update(void)
  {
    int raw_state = read_raw_state(pin);
    unsigned long cur_time = millis();

    // Reset if state changed
    if (raw_state != cur_state)
    {
      cur_state = raw_state;
      last_change_time = cur_time;
      consecutive_samples = 0;
      return DEBOUNCE_NO_EDGE;
    }

    // Need more samples
    if (consecutive_samples < required_samples)
    {
      consecutive_samples++;
      return DEBOUNCE_NO_EDGE;
    }

    // Need more time
    if ((cur_time - last_change_time) < delay_time)
    {
      return DEBOUNCE_NO_EDGE;
    }

    // Check if actual state changed
    if (raw_state != prev_stable_state)
    {
      prev_stable_state = cur_state;
      return raw_state ? DEBOUNCE_RISING_EDGE : DEBOUNCE_FALLING_EDGE;
    }

    return DEBOUNCE_NO_EDGE;
  }

private:
  // Read pin with inversion logic
  bool read_raw_state(int to_read)
  {
    bool raw = digitalRead(to_read);
    return inverted ? !raw : raw;
  }
};

// Global objects and variables
SoftwareDebounce debouncer(SWITCH_NUM, true, 50, 3);
bool led_state = false;

// Setup function
void setup()
{
  // Serial setup
  Serial.begin(115200);
  Serial.println("Software Debounce (Arduino): Starting program.");

  // LED setup
  pinMode(LED_NUM, OUTPUT);
  digitalWrite(LED_NUM, led_state);
}

// Main loop
void loop()
{
  debounce_edge_t debounce_result = debouncer.update();

  // Toggle LED on button press
  if (debounce_result == DEBOUNCE_RISING_EDGE)
  {
    led_state = !led_state;
    digitalWrite(LED_NUM, led_state);
    Serial.print("Software Debounce (Arduino): LED ");
    Serial.println(led_state ? "ON" : "OFF");
  }
  delay(1);
}