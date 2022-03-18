# -*- coding: utf-8 -*-
"""
Created on Mon Mar  7 16:15:06 2022

@author: NIcolas Hince DELL
"""

import numpy as np
import pickle

## original coordinate values with orientation start(Left)-Finish(Right)
# y1 [-3.625:3.625]
# x1 [0:12.623]

## needed coordinate values with orientation start(bottom)-Finish(top)
# y [0:70]
# x [0:50]

### make x and y all values matrices at step size 0.001

xyAllZeros = np.zeros([int(12.624/0.001),int(7.251/0.001)])
### start conversion
#to change orientation and fix negations ynew=y1+3.625 and xnew=-x1

pickle_load = open ("picklefile", "rb")
Rp = pickle.load(pickle_load)
Rpcop=Rp.copy()
Rp=list(Rp)
def convertDPI(Rp):
    for i in range(len(Rp)):   
        Rp[i]=list(Rp[i])
        for j in range(len(Rp[i])):  
            Rp[i][j]=list(Rp[i][j])
            Rp[i][j][0]= int(70*((-1*(Rp[i][j][0]-12.623))/12.623))
            Rp[i][j][1]= int(50*(((Rp[i][j][1])+3.625)/7.25))
    return Rp
convertDPI(Rp)
## full scale solution
# FullScale=[];        
# for i in range(len(Rp)):
#     FullScale.append(xyAllZeros)
#     for j in range(len(Rp[i])):
#         FullScale[i][Rp[i][j][0]][Rp[i][j][1]]=1
        
# NewScale=np.zeros([70,50]);
# final=[];
# final.append(NewScale)
# for j in range(len(FullScale[0])-1):
#     for k in range(len(FullScale[0][j])-1):
#         if FullScale[0][j][k]==1:
#             r=int((j/(len(FullScale[0])-1))*(len(NewScale)-1))
#             c=int((k/(len(FullScale[0][j])-1))*(len(NewScale[0])-1))
#             final[0][r][c]=1
        
### for multiple (takes too long)
# for i in range(len(FullScale)-1):
#     final.append(NewScale)
#     for j in range(len(FullScale[i])-1):
#         for k in range(len(FullScale[i][j])-1):
#             if FullScale[i][j][k]==1:
#                 r=int((j/(len(FullScale[i])-1))*(len(NewScale)-1))
#                 c=int((k/(len(FullScale[i][j])-1))*(len(NewScale[0])-1))
#                 final[i][r][c]=1                
            
                
        
    


 