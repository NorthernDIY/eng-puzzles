# eng-puzzles
## ECE4600 G15 2021/2022 Capstone Project Code & Design Files
**As prepared by: Rowen Guncheon, Nicolas Hince, David Stewart, Lin Zhan on March 18/2022**


# Server:
Python/Flask based server that runs on the Raspberry Pi (or other Linux system) and processing incoming data from the Maze Unit. Contains both acceleromter filtering and the full positioning system.

This is basically glue that combines project elements.

## Requires:
- Python 3.7
- - With the following libraries: numpy, pymysql, scipy, flask, colorama, csv, math, time, os, json, sys
