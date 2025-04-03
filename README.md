# STM32G474RE + SCD41 + SSD1306 Sensor Display

Simple project to start learning embedded development by creating a CO2/Temp/Humidity monitor inspired by Aranet products.

## Objectives

Interface with **SCD41 sensor** via I2C
Read and display CO2 (ppm), humidity (%), and temperature (C)
Present data on the SSD1306

## Hardware

- NUCLEO-G474RE development board
- SCD41 sensor
- SSD1306 OLED display

## Software

- stm32cubemx, (project generation, peripheral setup)
- vscode (ide/debugging)
- wsl2 (build enviroment)
- openocd (flashing, debugging)

## Skills developed

- stm32 architecture and peripheral setup (gpio, i2c, timers, etc.)
- build systems, debugging, git/version control
- debugging, logic analyzer