# Temperature Detector

## Description

This project demonstrates an obstacle detector system, that uses an DHT11 to identify temperature and humidity and indicate their presence using an LCD.

Based on FreeRTOS library, the project implements two tasks:

1. Temperature-Humidity Reading
    - The DHT11 collects temperature and humidity data from the enviroment.
 
2. LCD Display 
    - The data is sent to the LCD, which displays the temperature and humidity in real time.

A queue is used to transfer data from one task to another.
There is also an extra library-file called "lcdlib.h" and "lcdlib.c" which includes the functionalities of the LCD.

## Hardware Requirements and Circuit Schematic 

Below is a list of all the necessary components required for this project:

| Components        | Description       | Quantity        |
|----------------|-------------------|----------------|
|     Board     |     STM32 NUCLEO-F446RE     |     1     |
| Temperature Sensor   |     DHT11     |     1     |
| LCD 2004  | LCD with a PCF8574T | 1 |
| BreadBoard   | Standard solderless breadBoard | 1 |
| Jumper Wires   | Male-to-Male | As needed |
| LEDs   | Standard 5mm LEDs (Green, Red) | 2 |
| Resistors   | 100Ω (for LEDs) | 2 |
| Power Supply   | 5V via USB | 1 |

- The LEDs are connected via 100Ω resistors to GPIO PA1 and PA4.
- The DHT11 sensor has a 3.3V power supply, and the data pin is connected to GPIO PB0 for temperature and humidity readings.
- The LCD 2004 has a 5V power supply, the SDA pin is connected to GPIO PB6, and the SCL pin is connected to GPIO PB7.

## How to install and run this project ?
If you wish to test it, you need to download the entire project from GitHub. Then, setup the STM32CubeIDE and open the application files you've downloaded.

Note: Many of the files and libraries in this project are auto-generated by STM32CubeIDE. For the main implementation, refer to "main.c", "lcdlib.c" and "lcdlib.h" file. 
