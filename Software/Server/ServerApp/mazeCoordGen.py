# -*- coding: utf-8 -*-
"""
Created on Sun Jul 31 14:19:38 2022

@author: David
"""

#XDIM = 11.625 # Inches
#YDIM = 8 # Inches

#Slightly bigger than full maze
XDIM = 304 # mm
YDIM = 214 # mm
coords = []
for y in range(0, YDIM, 2):
    for x in range(0, XDIM, 2):
        coords.append([x,y])
print(coords)