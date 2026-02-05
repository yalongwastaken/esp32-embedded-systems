# Intro

ESP32 LED control implementations progressing from basic blocking code to RTOS multi-tasking with UART communication. Includes reusable logging utilities for future projects.

## Projects

### Arduino (`arduino/`)
| Project | Description |
|---------|-------------|
| `blinking_led_1/` | Simple blocking LED toggle |
| `blinking_led_2/` | Non-blocking LED toggle using timers |
| `blinking_led_3/` | Non-blocking LED control with serial speed adjustment |
| `logging/` | Reusable logging utilities |

### ESP-IDF (`esp-idf/`)
| Project | Description |
|---------|-------------|
| `blinking_led_1/` | Simple non-blocking LED toggle |
| `blinking_led_2/` | RTOS multi-tasking with UART and multiple LED states |
| `logging/` | Reusable logging utilities |

## Tools

- **Board:** ESP32-WROOM-32E
- **IDE:** VSCode + PlatformIO
- **Compatibility:** Arduino IDE, native `idf.py` toolchain