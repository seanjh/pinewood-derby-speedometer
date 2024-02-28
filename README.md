# Pinewood Derby Speedometer

Implements a simple speedometer for the 2024 Cub Scouts Pinewood Derby just because.

## Parts

- Adafruit HUZZAH32 - ESP32 Feather ([overview](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/overview), [product](https://www.adafruit.com/product/3405))
- Adafruit 7-Segment LED FeatherWings ([overview](https://learn.adafruit.com/adafruit-7-segment-led-featherwings/), [product](https://www.adafruit.com/product/3108))
- Adafruit LSM6DSOX + LIS3MDL FeatherWing - Precision 9-DoF IMU ([overview](https://learn.adafruit.com/st-9-dof-combo), [product](https://www.adafruit.com/product/4565))

## Setup

### Prerequisites

- `arduino-cli`
- `python3` (>=3.10)
- `make`
- (optional) `direnv`

### Instructions

- `arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json`
- `arduino-cli core update-index` - Update the core index
- `arduino-cli core install esp32:esp32` - Install the core for the ESP32
- `python3 -m pip install -r requirements.txt` - Install the ESP32 prerequisites
- Calibrate the sensors (see "How to Fuse Motion..." in references)

## References

 - [arduino-esp32](https://github.com/espressif/arduino-esp32) - Arduino core for the ESP32
 - [ESP32 Arduino Core’s documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/index.html)
 - [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
 - [Arduino CLI](https://arduino.github.io/arduino-cli/)
 - [FreeRTOS.org](https://www.freertos.org/implementation/a00002.html)
 - [FreeRTOS esp32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html)
 - [How to Fuse Motion Sensor Data into AHRS Orientation (Euler/Quaternions)](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions)
