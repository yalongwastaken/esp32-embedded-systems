# I2C LCD

ESP32 I2C communication implementations covering bus scanning and LCD1602 display interfacing. Features non-blocking state machine architecture, custom character rendering, and automated device detection.

## Projects

### Arduino (`arduino/`)
| Project | Description |
|---------|-------------|
| `i2c-lcd_basic/` | I2C bus scanning |
| `i2c-lcd_advanced/` | Potentiometer-controlled LED with brightness % on LCD |

### ESP-IDF (`esp-idf/`)
| Project | Description |
|---------|-------------|
| `i2c-lcd_basic/` | I2C bus scanning |
| `i2c-lcd_advanced/` | Potentiometer-controlled LED with brightness % on LCD |

## Hardware

- ESP32-WROOM-32E
- I2C LCD1602 (address: 0x27 or 0x3F)
- Pull-up resistors (2.2kΩ–4.7kΩ) if not on module
- Optional: potentiometer or photoresistor for advanced examples

## Tools

- **IDE:** VSCode + PlatformIO
- **Compatibility:** Arduino IDE, native `idf.py` toolchain