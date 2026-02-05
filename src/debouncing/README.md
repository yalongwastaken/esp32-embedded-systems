# Debouncing

ESP32 button debouncing implementations progressing from basic switch handling to interrupt-driven debouncing with RTOS integration. Techniques include finite state machines, hybrid polling/timing approaches, and interrupt-based solutions.

## Projects

### Arduino (`arduino/`)
| Project | Description |
|---------|-------------|
| `switch/` | Basic switch handling |
| `debounce_hybrid/` | Hybrid debouncing combining polling and timing |

### ESP-IDF (`esp-idf/`)
| Project | Description |
|---------|-------------|
| `switch/` | Basic switch handling with ESP-IDF GPIO drivers |
| `debounce_fsm/` | Finite state machine-based debouncing |
| `debounce_hybrid/` | Hybrid debouncing using FreeRTOS features |
| `debounce_interrupt/` | Interrupt-driven debouncing with timer callbacks |

## Tools

- **Board:** ESP32-WROOM-32E
- **IDE:** VSCode + PlatformIO
- **Compatibility:** Arduino IDE, native `idf.py` toolchain