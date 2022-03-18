# -*- coding: utf-8 -*-
"""
Created on Tue Mar  8 18:41:08 2022

@author: NIcolas Hince DELL
"""
import numpy as np
import pandas as pd
import csv



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
    #####F3 convertDPI ##added##
    for i in range(len(RealPaths)):   
        RealPaths[i]=list(RealPaths[i])
        for j in range(len(RealPaths[i])):  
            RealPaths[i][j]=list(RealPaths[i][j])
            RealPaths[i][j][0]= int(70*((-1*(RealPaths[i][j][0]-12.623))/12.623))
            RealPaths[i][j][1]= int(50*(((RealPaths[i][j][1])+3.625)/7.25))
    ###temp pickling###
    # filename="picklefile"
    # with open(filename, 'wb') as fp:
    #     pickle.dump(RealPaths,fp)
        
    for i in range(len(RealPaths)):
        RealSize.append(len(RealPaths[i]))
    return RealPaths        

def MakeListOfHertzNTimes(str):     
    AccelD=unpackCSVT2(str)
    AccelD.append([0])
    Hertz=[];
    Hertz.append([])
    Times=[];
    Times.append([])
    chk=1;
    k=0;
    s=0;
    HertzSum=0;
    for i in range(len(AccelD)):
        if AccelD[i][0]==chk:
            HertzSum=HertzSum+AccelD[i][6]
            s=s+1
        elif AccelD[i][0]!=chk :
            Times[k].append(round(AccelD[i-1][1]/100))
            Times.append([])
            Hertz[k].append(round(HertzSum/s))
            Hertz.append([])
            HertzSum=0
            s=0
            k=k+1; 
            chk=chk+1;
    Hertz.pop()    
    Times.pop()
    return Hertz, Times


RePaths=unpackCSVT1('MPosMore.csv')    
MakeListOfCoord(RePaths)
MakeListOfHertzNTimes('AccelMore.csv')