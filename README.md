# Temperature Detector

## Description

## Hardware Requirements and Circuit Schematic 

Below is a list of all the necessary components required for this project:

| Components        | Description       | Quantity        |
|----------------|-------------------|----------------|
|     Board     |     STM32 NUCLEO-F446RE     |     1     |
| Temperature Sensor   |     DHT11     |     1     |
| BreadBoard   | Standard solderless breadBoard | 1 |
| Jumper Wires   | Male-to-Male | As needed |
| LEDs   | Standard 5mm LEDs (Green, Red) | 2 |
| Resistors   | 100Ω (for LEDs) | 2 |
| Power Supply   | 5V via USB | 1 |

A detailed circuit schematic is also provided. This schematic show the connections between the components.

photo

- The LEDs are connected via 100Ω resistors to GPIO PA1 and PA4.
- The DHT11 sensor has a 3.3V power supply, and the data pin is connected to GPIO PB0 for temperature and humidity readings.

## How to install and run this project ?
If you wish to test it, you need to download the entire project from GitHub. Then, setup the STM32CubeIDE and open the application files you've downloaded.
