# eng-puzzles
##Serious Games - Summer Research 2022
**by David Stewart**

**For details of the software requirements for a particular portion of the project, please see the README.md file in the associated folder.**
# Summary of Software directory contents:
- **Addressing_Firmware**:  The initial Hall sensor address programming firmware that runs on the ESP32 SOC.
- **PacketSimulator**: Python application that emulates ESP32 data packet format (Used during development)
- **Server**:  Python/Flask based server that runs on the Raspberry Pi (or other Linux system) and processing incoming data from the Maze Unit
- **Visualization**: The data visualization php script that reads data from the SQL database and overlays this information on a web page.  (place in /var/www/html)
- **Archive**: Older versions of code.
- - **AllPaths**:  Algorithm used for generating a library of paths from a given input maze.  (For use with future machine learning work)
- - **Main_Firmware**:  The main firmware that runs on the ESP32 SOC within the Maze Unit.