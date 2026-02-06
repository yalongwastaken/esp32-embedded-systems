# FreeRTOS Introduction
ESP32 FreeRTOS implementations demonstrating real-time task scheduling, priorities, and multi-core execution.

## Projects

### Arduino (`arduino/`)
| Project | Description |
|---------|-------------|
| `freertos_basic/` | Dual LED blink with independent tasks |
| `freertos_advanced/` | Priority-based preemption with ISR notifications |

### ESP-IDF (`esp-idf/`)
| Project | Description |
|---------|-------------|
| `freertos_basic/` | Dual LED blink using native FreeRTOS APIs |
| `freertos_advanced/` | Priority demonstration with task pinning |

## Tools
- **Board:** ESP32-WROOM-32E
- **IDE:** VSCode + PlatformIO
- **Compatibility:** Arduino IDE, native `idf.py` toolchain