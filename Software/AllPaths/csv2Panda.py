# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 11:07:35 2022

@author: NIcolas Hince DELL
"""
import numpy as np
import pandas as pd
import csv
import pickle


## Function 1 
def SimCSV2RowsOfLists(str):
    SimPaths=[];
    SimSize=[];
    with open(str) as csv_file:
        reader = csv.reader(csv_file, delimiter='\n')
        rows = list(reader)
        rows.pop(0);
        
        for i in range(len(rows)):
            SimPaths.append(rows[i][0]) 
            SimPaths[i]=eval(SimPaths[i])
            SimSize.append(len(SimPaths[i]))
    csv_file.close()         
    return SimPaths, SimSize
def RealCSV2RowsOfLists(str):
    RePaths=[];
    RealPaths=[];
    RealSize=[];
    with open(str) as csv_file:
        reader = csv.reader(csv_file, delimiter='\n')
        rows = list(reader)

        for i in range(len(rows)):
            RePaths.append(rows[i][0]) 
            RePaths[i]=eval(RePaths[i])
            RePaths[i]=list(RePaths[i])
    csv_file.close()        
    RePaths=list(RePaths)
    k=0;
    chk=1;
    RealPaths.append([])
    for i in range(len(rows)):
        if RePaths[i][1]==chk:
            RealPaths[k].append((RePaths[i][3],RePaths[i][4]))
        else:
            RealPaths.append([])
            k=k+1;
            chk=chk+1;
    RealPaths=convertDPI(RealPaths)        
    for i in range(len(RealPaths)):
        RealSize.append(len(RealPaths[i]))
    return RealPaths, RealSize

## Function 3 (convert RealPaths DPI)
def convertDPI(Rp):
    for i in range(len(Rp)):   
        Rp[i]=list(Rp[i])
        for j in range(len(Rp[i])):  
            Rp[i][j]=list(Rp[i][j])
            Rp[i][j][0]= int(70*((-1*(Rp[i][j][0]-12.623))/12.623))
            Rp[i][j][1]= int(50*(((Rp[i][j][1])+3.625)/7.25))
    return Rp

def PathstoPandaDF(Paths,Size,Times,Hertz):
    dataframe=pd.DataFrame()        
    newdf1 = dataframe.assign(Paths=Paths)      
    newdf2 = newdf1.assign(Sizes=Size)
    newdf3 = newdf2.assign(Times=Times)      
    newdf4 = newdf3.assign(Hertz=Hertz)           
    dataSorted = newdf4.sort_values(by=['Sizes'], ascending=True)
    return dataSorted   

def unpackCSVT1(str): 
    RePaths=[];
    with open(str) as csv_file:
        reader = csv.reader(csv_file, delimiter='\n')
        rows = list(reader)
        
        for i in range(len(rows)):
            RePaths.append(rows[i][0]) 
            RePaths[i]=eval(RePaths[i])
            RePaths[i]=list(RePaths[i])
    csv_file.close()        
    ListOfListsT1=list(RePaths)
    return ListOfListsT1

def unpackCSVT2(str):
    Adata=[]    
    with open(str) as csv_file:
        reader = csv.reader(csv_file, delimiter='\n')
        rows = list(reader)
        for i in range(len(rows)):
            Adata.append(eval(rows[i][0]))
            Adata[i]=Adata[i][1:]
            Adata[i]=list(Adata[i])
            for j in range(len(Adata[i])):
                Adata[i][j]=eval(Adata[i][j])         
    csv_file.close()        
    ListOfListsT2=list(Adata)
    return ListOfListsT2

def MakeListOfCoord(RePaths):    
    RealPaths=[];
    RealSize=[];
    k=0;
    chk=1;
    RealPaths.append([])
    for i in range(len(RePaths)):
        if RePaths[i][1]==chk:
            RealPaths[k].append((RePaths[i][3],RePaths[i][4]))
        else:
            RealPaths.append([])
            k=k+1;
            chk=chk+1;
    for i in range(len(RealPaths)):   
        RealPaths[i]=list(RealPaths[i])
        for j in range(len(RealPaths[i])):  
            RealPaths[i][j]=list(RealPaths[i][j])
            RealPaths[i][j][0]= int(70*((-1*(RealPaths[i][j][0]-12.623))/12.623))
            RealPaths[i][j][1]= int(50*(((RealPaths[i][j][1])+3.625)/7.25))

    for i in range(len(RealPaths)):
        RealSize.append(len(RealPaths[i]))
    return RealPaths        

def MakeListOfHertzNTimes(str):     
    AccelD=unpackCSVT2(str)
    AccelD.append([0])
    Hertz=[];
    Times=[];
    chk=1;
    k=0;
    s=0;
    HertzSum=0;
    for i in range(len(AccelD)):
        if AccelD[i][0]==chk:
            HertzSum=HertzSum+AccelD[i][6]
            s=s+1
        elif AccelD[i][0]!=chk :
            Times.append(round(AccelD[i-1][1]/100))
            Hertz.append(round(HertzSum/s))
            HertzSum=0
            s=0
            k=k+1; 
            chk=chk+1;
    return Hertz, Times


xyAllZeros = np.zeros([int(12.624/0.001),int(7.251/0.001)])

# RePaths = unpackCSVT1('MPosMore.csv')    
# RealPaths = MakeListOfCoord(RePaths)
Hertz,Times = MakeListOfHertzNTimes('AccelMore.csv')

SimPaths,SimSize = SimCSV2RowsOfLists('ALLPATHS1.csv')
RealPaths,RealSize = RealCSV2RowsOfLists('MPosMore.csv') 
       
SimDataSorted = PathstoPandaDF(SimPaths,SimSize,0,0)
RealDataSorted = PathstoPandaDF(RealPaths,RealSize,Times,Hertz)

filename1="SimDataSorted"
with open(filename1, 'wb') as f1:
    pickle.dump(SimDataSorted,f1)    
f1.close()

filename2="RealDataSorted"
with open(filename2, 'wb') as f2:
    pickle.dump(RealDataSorted,f2) 
f2.close()    