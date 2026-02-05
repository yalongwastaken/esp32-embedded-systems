# PWM Control

ESP32 PWM implementations progressing from basic LED brightness control to multi-channel RGB systems with potentiometer input, ADC integration, and FreeRTOS task management.

## Projects

### Arduino (`arduino/`)
| Project | Description |
|---------|-------------|
| `pwm_basic/` | Single LED brightness control |
| `pwm_advanced/` | Potentiometer-controlled PWM with ADC |
| `pwm_multi-channel/` | RGB LED control with three synchronized PWM channels |

### ESP-IDF (`esp-idf/`)
| Project | Description |
|---------|-------------|
| `pwm_basic/` | LEDC peripheral PWM generation |
| `pwm_advanced/` | PWM control with ADC sampling and input validation |
| `pwm_multi-channel/` | RGB controller with FreeRTOS tasks and error queuing |

## Tools

- **Board:** ESP32-WROOM-32E
- **IDE:** VSCode + PlatformIO
- **Compatibility:** Arduino IDE, native `idf.py` toolchain