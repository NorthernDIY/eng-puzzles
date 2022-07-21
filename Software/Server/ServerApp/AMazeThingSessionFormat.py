# -*- coding: utf-8 -*-
"""
Created on Mon Jul  4 20:07:54 2022

@author: David
"""
import AMazeThing_Keys as KEY

#KEY items refer to the MSGPACK Tags defined in file above.  These are used during communications from ESP32<-->Server

#Tags stored in DB File 'MetaData'
ID = KEY.SESSIONID          #Session Id
IP = "IP"                   #Device IP Address
PT = "Port"                 #Session Communications Port assigned by server
ST = "StartTime"            #datetime.now() when session started
ET = "StopTime"             #datetime.now() when session ended this is based on LDT if a 'Bye' message is recieved at either shutdown or next device session start
HZ = "UsedHallZ"            #Did the server enable Hall Z data for this session
BPC = "BPC"                 #Corresponds to the number of packets that exceeded the single UDP packet message size on the ESP32 (The buffer there is only 1500 Char
ABS = KEY.ACCELBLOCKSIZE    #Accelerometer Data Block Size
AST = KEY.ACCELSAMPLETIME   #Accelerometer Sample Time Window (AKA Accel Period/Interval)
HBS = KEY.HALLBLOCKSIZE     #Hall Data Block Size
HST = KEY.HALLSAMPLETIME    #Hall Sample Time Window (AKA Hall Sample Period/Interval) *Input sample rate to the moving average filter
HMA = KEY.HALLMASIZE        #Hall Moving Average Size
HRT = KEY.HALLREPORTTIME    #Hall Report Time  (AKA Hall Report Period/Interval) (Related to HST, but samples the *current average* of the moving average)
NHS = KEY.NUMHALLSENSORS    #Number of Hall Sensors on Device
FWV = KEY.FWVER             #Firmware Version of Device
SC = "SessionComplete"      #If this isn't 1, the file didn't get closed properly or the session never ended

#RunTime Use only, not stored in DB File
FNAME = "FName" #FileName of DB
SQP = "SQLConn" #SQL 'Pipe'
SQC = "SQLCursor" #SQL Cursor
LHS = "LastHallPktStart"
LAS = "LastAccelPktStart"
LDT = "LastDataTime"

SRV_THREAD = "Thread"        #Session Thread Handle

#Some Debug Related things
TIME_SS = KEY.STARTSESSION   #ESP Micros() time stamp of session start (For Debugging purposes)
TIME_SE = KEY.ENDSESSION     #ESP Micros() time stamp of session start (For Debugging purposes)
TIME_STS = KEY.SENDSTS           #Recorded in metadata as this slightly increases MSGPACK payload size
TIME_KPT = "KeptPacketTimes" #0 = No tables {"HPacketTimes", "APacketTimes"} 1 = tables present (For Debugging Purposes)
TIME_HPT = "HTime"
TIME_HPTS = "Start"
TIME_HPTE = "End"
                
TIME_APT = "ATime"
TIME_APTS = "Start"
TIME_APTE = "End"

KPT_HTAGS = (TIME_HPT, TIME_HPTS, TIME_HPTE)
KPT_HTAGTYPES = ("INTEGER", "INTEGER", "INTEGER")
KPT_ATAGS = (TIME_APT, TIME_APTS, TIME_APTE)
KPT_ATAGTYPES = ("INTEGER", "INTEGER", "INTEGER")

#Session MetaData
SessionObject = {ID:-1, IP:"0.0.0.0",PT:7777, ST:-1, ET:-1, ABS:-1,
           AST:-1,HBS:-1, HST:-1, HMA:-1, HRT:-1, HZ:-1, NHS:-1, 
           FWV:"N/A", BPC:0, SC:False, TIME_STS: -1,TIME_KPT:-1, TIME_SS:-1,
           TIME_SE:-1, SQP:None, SQC:None, LHS:-1, LAS:-1,
           LDT:-1, FNAME:None, SRV_THREAD:None}


#SessionObjectTags: We skill the last 6 tags as they are not stored in the DB file
DBSessionObjectTags = (ID,
                       IP,
                       PT,
                       ST,
                       ET,
                       ABS,
                       AST,
                       HBS,
                       HST,
                       HMA,
                       HRT,
                       HZ,
                       NHS,
                       FWV,
                       BPC,
                       SC,
                       TIME_STS,
                       TIME_KPT,
                       TIME_SS,
                       TIME_SE)
DBSessionObjectTagTypes = ("INTEGER","TEXT","INTEGER","TEXT","TEXT","INTEGER","INTEGER","INTEGER","INTEGER","INTEGER",
                            "INTEGER","INTEGER","INTEGER","TEXT","INTEGER","INTEGER","INTEGER","INTEGER","INTEGER","INTEGER")

SessionObjectTags = (ID,IP,PT,ST,ET,ABS,AST,HBS,HST,HMA,HRT,HZ,NHS,FWV,BPC,SC,TIME_STS,TIME_KPT,TIME_SS,TIME_SE,SQP,SQC,SC,LHS,LAS,LDT, FNAME, SRV_THREAD)
