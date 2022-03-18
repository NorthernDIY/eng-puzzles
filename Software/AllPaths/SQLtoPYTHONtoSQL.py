# -*- coding: utf-8 -*-
"""
Created on Sat Jan 29 12:41:47 2022

@author: NIcolas Hince DELL
"""
import csv
import pandas as pd
import numpy as np
import pymysql
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, buttord, find_peaks


# db = pymysql.connect(host='localhost',
#                     user='root',
#                     password='raspberry',
#                     database='Puzzle')
# cursor =db.cursor()
# data = pd.read_sql("SELECT TOP(12000) * FROM ID",+\
#                    "SELECT TOP(12000) * FROM SessionID",+\
#                        "SELECT TOP(12000) * FROM TimeStamp",+\
#                            "SELECT TOP(12000) * FROM x_value",+\
#                                "SELECT TOP(12000) * FROM y_value",+\
#                                    "SELECT TOP(12000) * FROM z_value",db)

dataPacketSize=50 # the larger the data size the more the hertz is averaged over smaples
SamplingRate=400 # as per 100Hz sample/data rate of accelerometer

# array = np.genfromtxt(data, delimiter=",")
array = np.genfromtxt("accelerometer.csv", delimiter=",")
array = np.delete(array,(0), axis=0 )
array = array[50000:]
array = array[:50]
# ID=array[:,0];
# SessionID=array[:,1];
# TimeStamp=array[:,2];
x=array[:,2];
y=array[:,3];
z=array[:,4];
xyVect = (np.square(x) + np.square(y))**(0.5);
# xyzVect = (np.square(x) + np.square(y) + np.square(z))**(0.5);
N, Wn = buttord( [0.01885,0.1885], [0.01,19], 1, 10, False);    #buttord(wp, ws, gpass, gstop, analog=False)
                                                                #wp pass , ws stop, gain pass, gain stop,...
b,a=butter(N,Wn,'bandpass',analog=False) # N filter order, Wn For digital filters, Wn are in the same units as fs. By default, fs is 2 half-cycles/sample, so these are normalized from 0 to 1, where 1 is the Nyquist frequency
y=lfilter(b,a,xyVect) #apply filter on data

plt.plot(xyVect) #original Data
plt.plot(y) #filtered Data
plt.rcParams['figure.dpi'] = 600  #changes resolution for ploting
plt.rcParams['savefig.dpi'] = 600
plt.show()
hertz=np.zeros(int(np.size(array[:,1])/dataPacketSize))
j=0;
for i in range(int(np.size(y)/dataPacketSize)) :
    peaks = find_peaks(y[dataPacketSize*i:dataPacketSize*(i+1)])
    peaks = peaks[0]
    valleys = find_peaks(y[dataPacketSize*i:dataPacketSize*(i+1)]**(-1))
    valleys = valleys[0]
    numPeaks= (np.size(peaks)+np.size(valleys))/2
    hertzTemp = numPeaks*SamplingRate/dataPacketSize # as per 400Hz sample/data rate of accelerometer
    hertz[j] =hertzTemp
    
    j=j+1;
    i=i+dataPacketSize;
