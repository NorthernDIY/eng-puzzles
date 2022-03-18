# eng-puzzles
## ECE4600 G15 2021/2022 Capstone Project Code & Design Files
**As prepared by: Rowen Guncheon, Nicolas Hince, David Stewart, Lin Zhan on March 18/2022**

**For details of the software requirements for a particular portion of the project, please see the README.md file in the associated folder.**
# Summary of directory contents:
- ESP32/Addressing_Firmware:  The initial Hall sensor address programming firmware that runs on the ESP32 SOC.
- ML/AllPaths:  Algorithm used for generating a library of paths from a given input maze.  (For use with future ML work)
- ESP32/Main_Firmware:  The main firmware that runs on the ESP32 SOC within the Maze Unit.
- PacketSimulator: Python application that emulates ESP32 data packet format (Used during development)
- Server:  The required Python/Flask based server that runs on the Raspberry Pi (or other Linux system)
- Visualization: The data visualization php script that reads data from the SQL database and overlays this information on a web page.  (place in /var/www/html)
