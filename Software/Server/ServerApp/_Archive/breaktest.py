# -*- coding: utf-8 -*-
"""
Created on Mon Jun 13 13:17:04 2022

@author: David
"""

import keyboard
from time import sleep
while(True):
    if keyboard.is_pressed("q"):
        break
    sleep(5)
    print(".", end = "")

print("Exiting")