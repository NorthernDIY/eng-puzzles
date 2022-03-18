# eng-puzzles
## ECE4600 G15 2021/2022 Capstone Project Code & Design Files
**As prepared by: Rowen Guncheon, Nicolas Hince, David Stewart, Lin Zhan on March 18/2022**

**For details of the software requirements for a particular portion of the project, please see the README.md file in the associated folder.**

# Summary of Hardware directory contents:
- Hardware/MazeUnit:  STL files for 3D printing a completed maze unit (with ESP32 Sidecar)
- Hardware/PCB/ESP32_Subsystem: PCB design files for the ESP32 Subsystem board that sits in the sidecar adjacent to the maze unit.
- Hardware/PCB/POC: PCB Design files for the Proof Of Concept board featuring ALS31313 Sensors
- Hardware/PCB/Sensor_Block:  PCB design files for the modular (2x2) Hall sensor boards that sit beneath the maze floor.
- Hardware/Stylus:  STL files for 3D printing a stylus unit.

# Summary of Software directory contents:
- Software/ML/AllPaths:  Algorithm used for generating a library of paths from a given input maze.  (For use with future ML work)
- Software/ESP32/Main_Firmware:  The main firmware that runs on the ESP32 SOC within the Maze Unit.
- Software/PacketSimulator: Python application that emulates ESP32 data packet format (Used during development)
- Software/ESP32/Addressing_Firmware:  The initial Hall sensor address programming firmware that runs on the ESP32 SOC.
- Software/Server:  The required Python/Flask based server that runs on the Raspberry Pi (or other Linux system)
- Software/Visualization: The data visualization php script that reads data from the SQL database and overlays this information on a web page.  (place in /var/www/html)
