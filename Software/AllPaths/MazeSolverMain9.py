# -*- coding: utf-8 -*-
"""
Created on Sun Feb 27 16:22:53 2022
based on code from:
    Ahmed Khaled - 10111 silver badge5
    https://stackoverflow.com/questions/60908042/backtracking-in-a-recursive-maze-python
&
    https://www.delftstack.com/howto/python/python-append-to-csv/#:~:text=Append%20New%20Row%20to%20a%20CSV%20File%20in,to%20the%20CSV%20file%20using%20DictWriter.writerow%20%28%29.%20
&
    https://stackoverflow.com/questions/40855221/how-to-check-if-row-already-exists-in-csv

@author: Nicolas Hince

Project: Engineering Puzzles:
            Designing a Data Collection Device for Cognitive Assessment 

input: image of maze in - 70 by 50 pixel/black and white/bottom start, top finsh - format

output: random paths list in x,y coordiantes that solve the maze
"""
import sys
 
# the setrecursionlimit function is
# used to modify the default recursion
# limit set by python. Using this,
# we can increase the recursion limit
# to satisfy our needs
 
sys.setrecursionlimit(1000)

import random as rd
import numpy as np
import pandas as pd
import pygame
import csv
    
start_x = 31;
start_y = 69;
finish_x = 23;
finish_y = 0;
Up=[-1,0];
Down=[1,0];
Left=[0,-1];
Right=[0,1];
path=[];
ans=[]

def imgNumpy(str):
    pygame.init();
    image = pygame.image.load(str);
    pixel = pygame.PixelArray(image);

    color_array = [[image.unmap_rgb(pixel[x, y]) for x in range(
        0, image.get_width())] for y in range(0, image.get_height())];

    rgb_array = [[(column.r, column.g, column.b)
                  for column in row] for row in color_array];
    N = (255, 255, 255);
    rgb_array = [sub for sub in rgb_array if sub.count(N) != len(sub)];
    rgb_array = pd.DataFrame(rgb_array);
    img = np.array(rgb_array);  # img[y][x]

    return img

def copyANDclean(str):
    img = imgNumpy('Asset1.png');
    imgcop = img.copy();
    j = 0;
    while j < 70:
        i = 0;
        while i <= 49:
            if imgcop[j][i] < (200, 200, 200):
                imgcop[j][i] = 0;
                i = i+1;
            elif imgcop[j][i] != (0, 0, 0):
                imgcop[j][i] = 1;
                i = i+1;
        j = j+1
    imgcc = imgcop.copy()
    for k in range(3):
        m=0;
        while m<69:
            n=0;
            while n<49:
                if imgcc[m+1][n]==0:
                    imgcc[m][n]=0;
                n=n+1;
            m=m+1;
    for k in range(3):
        m=0;
        while m<69:
            n=0;
            while n<49:
                if  imgcc[m][n+1]==0:
                    imgcc[m][n]=0;
                n=n+1;
            m=m+1;
    
    return imgcc

def imgccReset(imgcc):
    j = 0;
    while j < 70:
        i = 0;
        while i <= 49:
            if imgcc[j][i]!=0:
                imgcc[j][i]=1;
            i=i+1;
        j=j+1;    
    return imgcc
 
imgcc=copyANDclean('Asset1.png')    

def backtrack(j, i):
    if imgcc[j][i] == 0 or imgcc[j][i] == 4:
        path.clear();
        imgccReset(imgcc);
        j=69;
        i=31;
    else:    
    
        if imgcc[j][i] == 3:
            imgcc[j][i] = 4;
        if imgcc[j][i] == 2:
            imgcc[j][i] = 3;
        if imgcc[j][i] == 1:
            imgcc[j][i] = 2;
    
        path.append((j, i));
        
        if j == finish_y and i == finish_x:
            ans.append(list(path));
            CheckNwrite2CSV('ALLPATHS.csv',ans)
            result=DoubleCheckRepeats()
            print(result[1])
            if len(ans)==1 and result[0]==True:
                return
            else:
                path.clear();
                imgccReset(imgcc);
                j=69;
                i=31;
        else:
            a,b,c,d = findweights(j, i);
            direction = rd.choices([Up,Down,Left,Right],weights=(a,b,c,d));
            a1,b1= direction[0];
            if valid(j+a1,i+b1):
                j=j+a1;
                i=i+b1;
                
        backtrack(j, i);
    return


def valid(j, i):
    return (j >= 0 and j <= 69) and (i >= 0 and i <= 49) 
def findweights(j,i):
    up=imgcc[j-1][i];
    left=imgcc[j][i-1];
    right=imgcc[j][i+1];
    if (j+1)>=70:
        down=0;
        up1=up*10**(-(up)**2);
        down1=down;
        left1=left*10**(-(left)**2);
        right1=right*10**(-(right)**2);
    else:
        down=imgcc[j+1][i];
        up1=up*10**(-(up)**up);
        down1=down*10**(-(down)**down);
        left1=left*10**(-(left)**left);
        right1=right*10**(-(right)**right);
                        
    return up1,down1,left1,right1

def CheckNwrite2CSV(str,ans):

    with open(str, 'r') as fr_object:
        reader_object = csv.reader(fr_object,)
        #print(reader_object)
        for row in reader_object:
            if row not in ans:
                add=True;
            else: add=False;    
        if add==True:
            with open(str, 'a', newline='') as fw_object:
                # Pass the CSV  file object to the writer() function
                writer_object = csv.writer(fw_object)
                writer_object.writerow(ans)  
                # Close the file object
                fw_object.close()

def DoubleCheckRepeats():
    with open('ALLPATHS1.csv','r') as file1:
        existingLines = [line for line in csv.reader(file1, delimiter=',')]
        file1.close()
    newSols = []

    with open('ALLPATHS.csv','r') as file2:
        reader2 = csv.reader(file2,delimiter=',')
        for row in reader2:
            if row not in newSols and row not in existingLines:
                newSols.append(row)
        if len(newSols)!=0:
            with open('ALLPATHS1.csv','a') as file1w:
                # Pass the CSV  file object to the writer() function
                writer_object = csv.writer(file1w, lineterminator = '\n')
                writer_object.writerow(newSols[0])
                file1w.close()
            result=[True,"new path found, path added to ALLPATHS1"]
        else: 
            result=[False, "Doubled path found, path ignored"]
    
    file2.close()
    return result
backtrack(start_y, start_x)
  
    
    
        
    
