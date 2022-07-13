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

SRV_THREAD = "Thread"

#Some Debug Related things
DBG_SS = KEY.STARTSESSION   #ESP Micros() time stamp of session start (For Debugging purposes)
DBG_SE = KEY.ENDSESSION     #ESP Micros() time stamp of session start (For Debugging purposes)
DBG_KPT = "KeptPacketTimes" #0 = No tables {"HPacketTimes", "APacketTimes"} 1 = tables present (For Debugging Purposes)
DBG_HPT = "HTime"
DBG_HPTS = "Start"
DBG_HPTE = "End"
                
DBG_APT = "ATime"
DBG_APTS = "Start"
DBG_APTE = "End"

KPT_HTAGS = (DBG_HPT, DBG_HPTS, DBG_HPTE)
KPT_HTAGTYPES = ("INTEGER", "INTEGER", "INTEGER")
KPT_ATAGS = (DBG_APT, DBG_APTS, DBG_APTE)
KPT_ATAGTYPES = ("INTEGER", "INTEGER", "INTEGER")

#Session MetaData
SessionObject = {ID:-1, IP:"0.0.0.0",PT:7777, ST:-1, ET:-1, ABS:-1,
           AST:-1,HBS:-1, HST:-1, HMA:-1, HRT:-1, HZ:-1, NHS:-1, 
           FWV:"N/A", BPC:0, SC:False, DBG_KPT:-1, DBG_SS:-1,
           DBG_SE:-1, SQP:None, SQC:None, LHS:-1, LAS:-1,
           LDT:-1, FNAME:None, SRV_THREAD:None}


#SessionObjectTags: We skill the last 6 tags as they are not stored in the DB file
DBSessionObjectTags = (ID,IP,PT,ST,ET,ABS,AST,HBS,HST,HMA,HRT,HZ,NHS,FWV,BPC,SC,DBG_KPT,DBG_SS,DBG_SE)
DBSessionObjectTagTypes = ("INTEGER","TEXT","INTEGER","TEXT","TEXT","INTEGER","INTEGER","INTEGER","INTEGER","INTEGER",
                            "INTEGER","INTEGER","INTEGER","TEXT","INTEGER","INTEGER","INTEGER","INTEGER","INTEGER")

SessionObjectTags = (ID,IP,PT,ST,ET,ABS,AST,HBS,HST,HMA,HRT,HZ,NHS,FWV,BPC,SC,DBG_KPT,DBG_SS,DBG_SE,SQP,SQC,SC,LHS,LAS,LDT, FNAME, SRV_THREAD)
