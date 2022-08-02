# -*- coding: utf-8 -*-
"""
Created on Sat Jul  2 20:49:05 2022

@author: David
"""

import sqlite3 as sql3
import MazeV1flip as MZ
import os
import AMazeThingSessionFormat as MDK
from AMazeThingSessionFormat import SessionObject as SOB
from AMazeThingSessionFormat import DBSessionObjectTags as SOB_T
from datetime import datetime as dt
from sys import argv as sysArgs
from sys import exit as sysExit
from pprint import pprint
from numpy import zeros, argmin, size as npsize, square as npsquare, add as npadd
import matplotlib.pyplot as plt

#JULY 29 EOD - Working on calculating BSC from sensor data in DB file
    #Next is to finally bring Rowen's Algorithm into this program and be sure
    #to modify parameters from source code to fit newly named parameters!
#JULY 30 EOD Position algorithm throws no errors, but does not work!
#outputs 0,0 always!

#JULY31 EOD 
"""Continue move to mm as units with sensor 42 as 0,0 location
    Normalize vecmag radius to 49mm (distance between sensors)
    filter radius >=49*2?
    import MazeV1Flip
"""


PRINTSESSIONTAGS = 1
VERBOSE = 1
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

timeFmtString = "%Y-%m-%d %H:%M:%S.%f"
startTime = 0
endTime = 0
elapsedTime =0

HallBaseLineNoise = [0,0,0]
BSX = 0
BSY = 0
BSZ = 0
BSC = 0

lastHTime = 0 #Time Index of last Hall Data Sample

MazeCoordinates = MZ.MAZECOORDS.copy()
coords = []
SENSORDIST=MZ.SENSORDIST # distance between sensors
SWITCHDIST=MZ.SWITCHDIST #distance between sensor 31 and the microswitch
BOARDSEP=MZ.BOARDSEP #z distance between magnet and sensor boards while stylus is in the maze
IRRELEVENT=MZ.IRRELEVENT #used to eliminate data from far away sensors

FILTER=MZ.FILTER #used for coordinate filtering
ALTFILTER=MZ.ALTFILTER #used for elucidean filtering (alternative to coordinate filtering, don't use both)
mazeSize=MZ.NumValidPoints

distances = []


#Positioning System Variables
HallBaseLineNoise=[0,0,0] #x,y,z
BSCONST=0 #biot savart constant
lastPos=[100,100]
trials=0

sensorVec = []
sensorVecMag = []

def genAllCoords():
    global coords, MazeCoordinates, mazeSize
    XDIM = 314 # mm
    YDIM = 214 # mm
    coords = []
    for y in range(-10, YDIM-10, 2):
        for x in range(-10, XDIM-10, 2):
            coords.append([x,y])
    MazeCoordinates = coords
    mazeSize = len(coords)
    
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
    except Exception as e:
        print("Here's your error message!")
        print(e)
    if sqlConn != None:
        cursor = sqlConn.cursor()
        mySession = SOB.copy()
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
    
    
def GetSessionParams():
    global mySession, startTime, endTime, elapsedTime
    print("Reading Session Parameters... ")
    cursor = mySession[MDK.SQC]
    for tagPos in range(0,len(SOB_T)):
        tag =SOB_T[tagPos]
        cursor.execute(qPrefix + tag + qPostfix)
        mySession[tag]= cursor.fetchone()[0]
        if PRINTSESSIONTAGS or VERBOSE:
            print("\t"+ tag + " : "+ str(mySession[tag]))
    #print(mySession)
    startTime = dt.strptime(mySession[MDK.ST],timeFmtString)
    endTime = dt.strptime(mySession[MDK.ET],timeFmtString)
    elapsedTime = endTime - startTime
    if (not mySession[MDK.SC]):
        print("*WARNING* Session was not marked as completed! Probable data corruption!")
    print("\n\tElapsed Session Time: " + str(elapsedTime)+"\n")
    print
    return
    
    
    
def CalculateBioSavartRTC():
    global BSX, BSY, BSZ, BSC
    print("Calculating BioSavart RuntTime Calibration Constant:")
    cursor = mySession[MDK.SQC]
    sqlQueryBS = "SELECT * FROM HBioSav;"
    cursor.execute(sqlQueryBS)
    BSX, BSY, BSZ = cursor.fetchone()
    if VERBOSE:
        print("\tUsing BSX: " + str(BSX))
        print("\tUsing BSY: " + str(BSY))
        print("\tUsing BSZ: " + str(BSZ))
    #Use HallBaseLineNoise[0][0 through 2] as Sensor 31 is in the first column and therefore uses the average noise
    bioSavTemp=((float(BSX)-HallBaseLineNoise[14][0])**2 +(float(BSY)-HallBaseLineNoise[14][1])**2+(float(BSZ)-HallBaseLineNoise[14][2])**2)**0.5
    BSC=(((SWITCHDIST**2+BOARDSEP**2)**0.5)**3)*bioSavTemp #Biot Savart formula based calculation
    print("\tBioSavart RunTime Calibration Constant: %4.4f" %BSC)

#Look at all the time entries in Hall, if there are missing entries attempt to
#fill missing row with linearly interpolated data, and note in the db file that
#this occured.  Returns True if no data insertions made, false otherwise
def CheckHallContinuity():
    global lastHTime
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    print("Checking Hall Data for Missing time steps... ",end ="")
    tIndex = 0
    tStep = mySession[MDK.HRT]#Hall sample time in ms
    sqlGetLastRowQuery = "SELECT time FROM Hall ORDER BY time DESC LIMIT 1;"
    cursor.execute(sqlGetLastRowQuery)
    lastHTime = cursor.fetchone()[0] + mySession[MDK.HRT] #add the last one in since we zero index
    
    skippedTimes = []
    atLeastOneMissing = False
    for tIndex in range(0, lastHTime, tStep):
        sqlTimeQuery = "SELECT time FROM Hall WHERE time =" + str(tIndex) + ";"
        cursor.execute(sqlTimeQuery)
        item = cursor.fetchone()
        if item ==None:
            skippedTimes.append(tIndex)
            atLeastOneMissing = True
        #print(item)
    
    #Fill with linearly interpolated data from previous and next rows, also record where this happened
    if atLeastOneMissing:
        if VERBOSE:
            print("Missing Hall Data @" + str(skippedTimes))
        for tIndex in skippedTimes:
            sqlGetPreviousRowQuery = "SELECT * FROM Hall WHERE time =" + str(tIndex-mySession[MDK.HRT]) + ";"
            sqlGetNextRowQuery = "SELECT * FROM Hall WHERE time =" + str(tIndex+mySession[MDK.HRT]) + ";"
            cursor.execute(sqlGetPreviousRowQuery)
            previousRow = cursor.fetchone()
            cursor.execute(sqlGetNextRowQuery)
            nextRow = cursor.fetchone()
            if (nextRow!=None) & (previousRow !=None):
                if VERBOSE:
                    print("\tFill misssing with Lin. Int. data @" + str(tIndex))
                interpolatedData = list()
                for data1, data2 in zip(nextRow, previousRow):
                    interpolatedData.append((data1 + data2)//2)
                #Cast to tuple means string translation puts () instead of []
                intData = tuple(interpolatedData)
                sqlAddIntData = "INSERT INTO Hall VALUES" + str(intData)
                cursor.execute(sqlAddIntData)
                sqlConn.commit()
        sqlAddMissingTable = "CREATE TABLE IF NOT EXISTS HMissedTimes(Time INTEGER);"
        cursor.execute(sqlAddMissingTable)
        sqlConn.commit()
        for missed in skippedTimes:    
            sqlAddMissedTimes = "INSERT INTO HMissedTimes VALUES (" + str(missed) +");"
            cursor.execute(sqlAddMissedTimes)
        sqlConn.commit()
    if VERBOSE:
        if atLeastOneMissing:
            print("Missing data points found, interpolation applied!\n")
        else:
            print("All good!\n")
    return (not atLeastOneMissing)

def CalculateBaseLineNoise():
    global HallBaseLineNoise
    
    HallBaseLineNoise = []
    for i in range(mySession[MDK.NHS]):
        HallBaseLineNoise.append(0);
    
    cursor = mySession[MDK.SQC]
    #sqlConn = mySession[MDK.SQP]
    #from Algorithm V1 we want columns 3 through 6 (not zero indexed)
    NoisySensorList = MZ.sensCol3.copy()
    NoisySensorList.extend(MZ.sensCol4.copy())
    NoisySensorList.extend(MZ.sensCol5.copy())
    NoisySensorList.extend(MZ.sensCol6.copy())
    #print(NoisySensorList)
    Nx, Ny, Nz = 0.00000, 0.000000, 0.00000
    
    AveragedNoiseSensorList = MZ.sensCol1.copy()
    AveragedNoiseSensorList.extend(MZ.sensCol2.copy())
    
    for sensor in NoisySensorList:
        sqlSensorQuery = "SELECT * from HNoise WHERE senseNum = " + str(sensor) + ";"
        cursor.execute(sqlSensorQuery)
        NData = cursor.fetchone()
        HallBaseLineNoise[sensor] = list(NData[1:])
        Nx += float(NData[1])
        Ny += float(NData[2])
        Nz += float(NData[3])
        #print(NData)
    
    BaseLineNoise = [Nx/16.0, Ny/16.0, Nz/16.0]
    for sensor in AveragedNoiseSensorList:
        HallBaseLineNoise[sensor] = BaseLineNoise
    
    print(HallBaseLineNoise)
    #if VERBOSE:
    #    print("Calculated Baseline Noise: " + str(HallBaseLineNoise)+"\n")
    
def grabHallData(timeStep):
    cursor = mySession[MDK.SQC]
    sqlHallDataQuery = "SELECT * FROM Hall WHERE TIME = " + str(timeStep) + ";"
    cursor.execute(sqlHallDataQuery)
    HallDataAtTimeStep = cursor.fetchone()[1:]#Trim off Time
    return HallDataAtTimeStep

def CalculatePosition(timeStep):
    global lastPos, trials,ax,plt,sensorVec,sensorVecMag
    NumSensors = mySession[MDK.NHS]
    #distances = zeros((NumSensors,mazeSize))
    HallData = grabHallData(timeStep)
    #print(HallData)
    #print(len(HallData))
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    #print("Calculating position at session time %d" %timeStep)
    weight=zeros((1,len(MazeCoordinates)))
    #funcTimeStart = time.time()
    sqlCreatePositionResultQuery = "CREATE TABLE IF NOT EXISTS MZPos(Time INTEGER,X_POS DECIMAL,Y_POS DECIMAL ,MZ_POINTINDEX INTEGER ,MZ_VERSION DECIMAL)"
    cursor.execute(sqlCreatePositionResultQuery)
    sqlConn.commit()
    sqlPositionDataInsertQuery= "INSERT INTO MZPos VALUES(?, ?, ?, ?, ?);"
    mazeWeights=zeros((1,mazeSize)) #creates/clears vector of weights
    sensorVec = []
    sensorVecMag = []
    for sensor in range(0, NumSensors): # iterate through sensors
        xData = HallData[sensor]
        yData = HallData[sensor + 24]
        zData = HallData[sensor + 48]
        
        tx = (float(xData) - float(HallBaseLineNoise[sensor][0]) + 0.000001)**2
        ty = (yData - HallBaseLineNoise[sensor][1]+ 0.000001)**2
        tz = (zData - HallBaseLineNoise[sensor][2]+ 0.000001)**2
        #print("x " + str(tx))
        #print("y " + str(ty))
        #print("z " + str(tz), flush=True)
        vec=(tx + ty + tz)**0.5
        #print("Vec " + str(vec), flush=True)
        #plt.Circle((MZ.POS_SENSORS[sensor]),vec)        
        #print (BSC/vec, flush=True)
        vecmag=(BSC/vec)**0.33 #Biot Savart Law based calculation
        #plt.show()
        sensorVec.append(vec)
        sensorVecMag.append(vecmag)
        
        if vecmag<=MZ.IRRELEVENT: #removes sensors too far from the magnet
            weight[0]=(distances[sensor]-vecmag)**2
            mazeWeights[0]=npadd(mazeWeights[0],weight[0])
            sensorVec[sensor] = 0
            sensorVecMag[sensor] = 0
    minLoc=argmin(mazeWeights) #find minimum error(weight)
    
        
    point=[-MazeCoordinates[minLoc][0],MazeCoordinates[minLoc][1]]
    
    #Filtering using comparison to last point....
    #compare based on last point (removes outliers)
    """if (lastPos[0]==100)or(lastPos[0]-FILTER<=MazeCoordinates[minLoc][0]<=lastPos[0]+FILTER)or(lastPos[1]-FILTER<=MazeCoordinates[minLoc][1]<=lastPos[1]+FILTER):
    #alternate filtering
    #if ((lastPos[0]==100) or (((MazeCoordinates[minLoc][0]-lastPos[0])**2+(MazeCoordinates[minLoc][1]-lastPos[1])**2)**0.5)<MZ.ALTFILTER):
        sqldata= timeStep, point[0], point[1],"N/A", "N/A"
        #cursor.execute(sqlPositionDataInsertQuery,sqldata)
        #print(sqlPositionDataInsertQuery)
        print(sqldata)
                #print(sql)
        lastPos=MazeCoordinates[minLoc]
        trials=0
    else:
        trials=trials+1
        if trials>=3:
            sqldata= timeStep, point[0], point[1],"N/A", "N/A"#-1,MZ.MAZEVERSION
            #cursor.execute(sqlPositionDataInsertQuery,sqldata)
            #print(sqlPositionDataInsertQuery)
            print(sqldata)
            #print(sql)
            lastPos=MazeCoordinates[minLoc]
            trials=0
    """
    #note x is inverted on plot!??  Why tho....?
    pColourMultiplier = float(timeStep)/lastHTime
    pColour = (0.25+0.25*pColourMultiplier, pColourMultiplier, 0.25+ 0.25*pColourMultiplier)
    pAlpha = 0.33 +pColourMultiplier*0.33
    ax.plot(-point[0], point[1], marker = "o", markersize = 10, markeredgecolor = "black", markerfacecolor=pColour, alpha=pAlpha)
    
    
def genSensorVectorDistances():
    global distances
    distances = zeros((mySession[MDK.NHS],mazeSize))
    for i in range(mySession[MDK.NHS]):
        for j in range(mazeSize):
            distances[i][j]=((MazeCoordinates[j][0]-MZ.POS_SENSORS[i][0])**2+(MazeCoordinates[j][1]-MZ.POS_SENSORS[i][1])**2 + MZ.BOARDSEP**2)**0.5

def PlotPositions(ListOfPositions, overlayImage, boolSaveFile, saveFileName):
    print("Overlaying positions on image");
    if boolSaveFile:
        print("Saving to file %s"%saveFileName)

def prepPlot():
    global ax,img
    ax.imshow(img, extent = [0, 305, 0, 203])
    #Plot sensor locations
    for point in MZ.POS_SENSORS:
        ax.plot(point[0], point[1], marker = "s", markersize = 10, markeredgecolor = "grey", markerfacecolor="red", alpha=0.5)
#main:
print("AMazeThing Session Digestion Application")
print("Version: %s"%VERSIONSTRING)
FileName = GetArguments()
Opened = OpenDB(FileName)
plt.rcParams["figure.figsize"] = 12, 8
plt.rcParams["figure.autolayout"] = True
img = plt.imread("MazeBWG.png")
fig, ax = plt.subplots()
prepPlot()
#These extents are trial & Error ....but maybe indicates something is wrong with the coordinate scale in v1!!
#ax.imshow(img, extent = [-0.125, 13, -4.66, 4.66])

#Plot valid path coordinate entries
#for point in MazeCoordinates:
#    ax.plot(-point[0], -point[1], marker = "o", markersize = 5, markeredgecolor = "green", markerfacecolor="green", alpha=0.25)



if Opened:
    genAllCoords()
    GetSessionParams()

    CheckHallContinuity()
    CalculateBaseLineNoise()
    CalculateBioSavartRTC()
    genSensorVectorDistances()
    #print(distances)
    for tS in range(0,lastHTime,100):
        #plt.rcParams["figure.figsize"] = 12, 8
        #plt.rcParams["figure.autolayout"] = True
        #img = plt.imread("MazeBWG.png")
        #fig, ax = plt.subplots()
        #prepPlot()
        CalculatePosition(tS)
        #plt.show()
        #print("FFFS:")
    #CalculatePosition(100)
    #CalculatePosition(400)
    #CalculatePosition(1900)
    plt.show()
    #plot 0,0 point
    #ax.plot(0, 0, marker = "*", markersize = 20, markeredgecolor = "black", markerfacecolor="yellow")
    
    index = 0
    print(sensorVecMag)
    for point in MZ.POS_SENSORS:
        ax.plot(point[0], point[1], marker = "o", markersize = (sensorVecMag[index]*30), markeredgecolor = "blue", markerfacecolor="none", alpha=0.50)
        index +=1
    #CalculatePosition(11000)
    #for point in MZ.POS_SENSORS:
    #    ax.plot(-point[0], -point[1], marker = "o", markersize = (sensorVecMag[index]*100), markeredgecolor = "red", markerfacecolor="none", alpha=0.50)
    #    index +=1
    #plt.show()
CloseDB()
#print(mySession)