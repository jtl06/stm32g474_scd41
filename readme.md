# CO2, Humidity, and Temperature Display and Logger

- Simple project to start learning embedded development creating a CO2/Temp/Humidity monitor inspired by Aranet products.

## Objectives

- Interface with SCD41 sensor via i2c
- display CO2 (ppm), humidity (%), and temperature (C) on the SSD1306 screen via i2c
- write to SD card over SPI interface
- Learn various common protocols at an intimate level by implementing them (I2c, SPI, USB)

## Hardware

- NUCLEO-G474RE development board
- SCD41 sensor
- SSD1306 OLED display
- SD (or micro SD card) to SPI breakout board

## Software and tools used

- stm32cubemx, openocd
- wsl, vscode
- saelae logic analyzer

## TODO
- Write I2C driver (shouldnt be to bad?)
- Write SPI driver (moderately difficult?)
- Add USB connectivity (need board with USB... maybe nucleo stm32C071?)
- Write USB driver from scratch (difficult?)
- Make custom PCB and case (doable... battery powered?)