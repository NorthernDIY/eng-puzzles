# -*- coding: utf-8 -*-
"""
Created on Sat Jul  2 20:49:05 2022

@author: David
"""

import sqlite3 as sql3
import os
import AMazeThingSessionFormat as MDK
from AMazeThingSessionFormat import SessionObject
from AMazeThingSessionFormat import DBSessionObjectTags as SOT
from dbmetatags import qTags
import datetime as dt
from sys import argv as sysArgs
from sys import exit as sysExit
from pprint import pprint

#SQL Things
sqlConn = None
cursor = None

mySession= None

db_RawDir = '/RawDat/'
dir_reports = '/Reports/'
dir_reportData = '/Reports/Data'
intakeFile = ''

VERSIONSTRING = '0.01a'

qPrefix = "SELECT "
qPostfix = " FROM MetaData;"


def GetArguments():
    args = sysArgs
    alen = len(args)
    valid = False
    fileName = ''
    if alen==3:
            if args[1] == "--in":
                fileName = args[2]
                print("Digesting file: %s" %intakeFile)
                valid = True
            elif ~valid:
                    print("Usage: digest.py -i 'InputFileName.db'")
    return fileName

def OpenDB(fileName):
    filePathName = os.getcwd() + db_RawDir + fileName
    print("Opening Session File: %s..."%filePathName)
    global sqlConn, cursor, mySession
    try:
        sqlConn = sql3.connect(filePathName, timeout = 10)
    except Error as e:
        print(e)
    if sqlConn != None:
        cursor = sqlConn.cursor()
        mySession = SessionObject.copy()
        mySession[MDK.SQP] = sqlConn
        mySession[MDK.SQC] = cursor
        mySession[MDK.FNAME] = filePathName #Stored temporarily with the session object
        mySession.pop(MDK.SC)
        mySession.pop(MDK.LHS)
        mySession.pop(MDK.LDT)
        mySession.pop(MDK.LAS)
        return True
    else:
        return False

def CloseDB():
    print("Closing Session File: %s..."%mySession[MDK.FNAME])
    mySession[MDK.SQP].close()
    

    
def GetNoiseParams(sqlConn):
    print("Reading Baseline Noise...")
    
def GetSessionParams():
    print("Reading Session Parameters...")
    cursor = mySession[MDK.SQC]
    for tagPos in range(0,len(qTags)):
        cursor.execute(qPrefix + qTags[tagPos] + qPostfix)
        mySession[SOT[tagPos]]= cursor.fetchone()[0]
    print(mySession)
    return
    #UseHZ = (thing==1)
    print("Session Properties:")
    print("\tStart Time: ", end = "")
    #print(SessionStartTime)
    print("\tSession End Time: ", end = "")
    #print(SessionEndTime)
    print("\tTotal time elapsed: ", end = "")
    #print(SessionEndTime- SessionStartTime)
    print("\tDevice Firmware Version:", end = "")
    #print(DeviceFirmwareVersion)
    print("\tDevice IP:", end = "")
    #print(DeviceIP)
    print("\tBad Packet Count:", end ="")
    #print(BPC)
    
    print("\tHall Sensor Parameters:")
    print("\t\tHall Sensors:", end = "")
    #print(NumSensors)
    print("\t\tHall Block Size:", end = "")
    #print(HBSz)
    print("\t\tHall Sample Period:", end = "")
    #print(HST)
    print("\t\tHall Report Period:", end = "")
    #print(HRT)
    print("\t\tHall MA Size:", end = "")
    #print(HallMAsz)
    print("\t\tHall Uses Z:", end = "")
    #print(UseHZ)
    
    print("\tAccelerometer Parameters:")
    print("\t\tAccel Block Size:", end = "")
    #print(ABSz)
    print("\t\tAccel Sample Period:", end = "")
    #print(AST)
    
    
def GetBioSavartRTC(sqlConn):
    print("Reading BioSavart RunTimeConstants...")
    BSX = 0.0
    BSY = 0.0
    BSZ = 0.0
    return BSX,BSY,BSZ

def CheckHallContinuity(sqlConn):
    print("Checking Hall Data for Missing time steps...")
    #return (LIST OF MISSING ENTRIES)

def CalculateBaseLineNoise():
    print("Calculating Baseline Noise Corrections for Hall Sensors...")

def CalculatePosition(HallData, timeStep):
    print("Calculating position at session time %d" %timeStep)
    positionIndex=0
    
    return(positionIndex)

def PlotPositions(ListOfPositions, overlayImage, boolSaveFile, saveFileName):
    print("Overlaying positions on image");
    if boolSaveFile:
        print("Saving to file %s"%saveFileName)
        
#main:
print("AMazeThing Session Digestion Application")
print("Version: %s"%VERSIONSTRING)
FileName = GetArguments()
Opened = OpenDB(FileName)
if Opened:
    GetSessionParams()
CloseDB()
print(mySession)