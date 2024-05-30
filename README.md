# About
This project implements a driver for the **BME280** sensor (pressure, temperature, humidity) using ESP-IDF's SPI master driver. The project is written in C++ and can be easily imported by simply cloning this project to the ESP-IDF's project folder.
The example for the application code can be found in `spi.cpp`, which has implemention of both Forced and Normal mode (more infromation can be found in the reference manual).

### PINOUT
---
The project was tested and ran on the ESP32 WROOM32D, and a pinout diagram can be found [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/_images/esp32-devkitC-v4-pinout.png).

### Reference Manual
---
The reference manual for the sensor can be found [here](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf).
