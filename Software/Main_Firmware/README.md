# eng-puzzles
## ECE4600 G15 2021/2022 Capstone Project Code & Design Files
**As prepared by: Rowen Guncheon, Nicolas Hince, David Stewart, Lin Zhan on March 18/2022**

# Main_Firmware:  The main firmware that runs on the ESP32 SOC within the Maze Unit.

## Requires:
- Visual Studio Code:
- - With PlatformIO plugin installed
- - - With ESP32 Board support package installed

- Libraries & Frameworks:
- - arduino-esp32 (https://github.com/espressif/arduino-esp32): Enables arduino framework to run on the ESP32 SOC.
- - ArduinoJSON (https://github.com/bblanchon/ArduinoJson): Enables serialization of data to JSON format.
- - SparkFun LIS2DH12 Arduino Library (https://github.com/sparkfun/SparkFun_LIS2DH12_Arduino_Library):  Provides support for the LIS2DH12 accelerometer.