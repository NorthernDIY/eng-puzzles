#!/bin/python3
#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, sqlite3,os
import AMazeThing_Keys as KEY
import AMazeThingSessionFormat as MDK
from AMazeThingSessionFormat import sessionObject
from msgpack import loads as dec_msgpack
from msgpack import dumps as enc_msgpack
from sys import argv as sysArgs
from sys import exit as sysExit
from datetime import datetime

from enum import Enum
from pprint import pprint
import json

packet = None
addresses= None

#Things you might want to change
#Network Settings (Defaults)
localIP_d     = "127.0.0.1"# Localhost for now
localPort_d   = 20002
bufferSize_d  = 20000

"""
Todo:	-(COMPLETE JULY 4/2022)     Fix missing RawDat directory error
		-Detect current OS vs Last OS at startup and autoreset if different (Helpful for development)
		-Watch Session ID of incoming packet and discard if not associated with IP (Basic anti-spoofing)
		-Figure out how to close out sessions after some time of no activity --or- when next session started by same IP
		-Netifaces get all interfaces, no joke stop skipping things randomly
		
"""
#Run Time Variables
#FileName Things
db_RawDir = '/RawDat/'
db_fName = '.' + db_RawDir + 'MZRaw_S' #Session db prefix
cfg_fName = 'ServerConfig.json' #Filename of settings storage

#Network setting variables
localIP     = localIP_d
localPort   = localPort_d
bufferSize  = bufferSize_d

DBG_CLEAR_EVERY_RUN = 1
DBG_DEFIP = "192.168.137.1"
DBG_DEFPT = 20002
DBG_CAP_PKT_TIMES =1

Sessions = []

#Client Device parameter Adjustments
#At session start, server sends the following parameters to the ESP32
ServerWantsHZ = 1	#(Sent to ESP32) 1 = Send Hall Z data, 0 = Don't send
ServerWantsSTS = 1  #(Sent to ESP32) 1 = Send sample time start/end time stamps, 0 = don't send
HST = 10 			#(Sent to ESP32)Hall Sensor Sampling Period
HRT = 100			#(Sent to ESP32)Hall Sensor Report time - [Interval in which MA-Filtered Hall data is sent]
HMA = 20	#MAX=60	#(Sent to ESP32)Hall Sensor # of entries in Moving Average Filter
AST = 40 			#(Sent to ESP32)Accelerometer Sampling Interval

#Session Things
sessionID = 0 #Global SessionID increments every Hello and is saved/restored from settings file

class msgType(Enum):
    JUNK= 0
    START =1
    END = 2
    HALL = 3
    ACCEL = 4
    CAL_HALL = 5
    CHECK = 6

def unpack(UDP_packet):
    message = None
    try:
        message = dec_msgpack(UDP_packet)
    except:
        print("Failed to decode MSGPACK")
        print("Contents: ", end = "")
        print(UDP_packet)
        message = None
    finally:
        return message

#Determine message type based on presence of specific Keys
def getMsgType(message):
    if (message != None):
        if message.__contains__(KEY.STARTSESSION):
            return msgType.START
        elif message.__contains__(KEY.HALLTIME):
            return msgType.HALL
        elif message.__contains__(KEY.ACCELTIME):
            return msgType.ACCEL
        elif message.__contains__(KEY.ENDSESSION):
            return msgType.END
        elif message.__contains__(KEY.BIOSAVX):
            return msgType.CAL_HALL
        elif message.__contains__(KEY.SRVCHECK):
            return msgType.CHECK
    else:
        print("Bad Packet Type!")
        return msgType.JUNK

def SaveSettings():
    #store vars as dict for easy JSON Load/Dump
    cfgVars = {"IP": localIP, "Port": localPort, "bSize": bufferSize, "SID":sessionID}
    with open(cfg_fName, 'w') as outfile:
        json.dump(cfgVars, outfile)

def LoadSettings():
    global sessionID, localIP, localPort, bufferSize
    if os.path.exists(cfg_fName):
        with open(cfg_fName) as json_file:
            cfgVars = json.load(json_file)
        sessionID = cfgVars["SID"]
        localIP = cfgVars["IP"]
        localPort = cfgVars["Port"]
        bufferSize = cfgVars["bSize"]
        print("Last Session # loaded from file: %d" %sessionID)   
    else:
        print("No Settings file found, using defaults")


def newSQLdb(mySession): #Setup the session specific DB
    global sqlConn, cursor
    fName = db_fName + str(mySession[MDK.ID]) + '.db'
    sqlConn = sqlite3.connect(fName, timeout = 10)
    cursor = sqlConn.cursor()
    mySession[MDK.SQP] = sqlConn
    mySession[MDK.SQC] = cursor
    UseHZ = mySession[MDK.HZ]
    NumHallSensors = mySession[MDK.NHS]
    
    createMetaDataTable = "CREATE TABLE MetaData(SessionID INTEGER PRIMARY KEY, IP TEXT, StartTime TEXT, StopTime TEXT, ABS INTEGER, AST INTEGER, HBS INTEGER, HST INTEGER, HallMASize Integer, HRT Integer, USEHZ Integer, NumSensors Integer, FWVer TEXT, BadPacketCount Integer, ESP_START Integer, ESP_STOP Integer);"
    if DBG_CAP_PKT_TIMES:
        createHPktTimesTable = "CREATE TABLE HPacketTimes(HTime Integer, STime Integer, ETime Integer);"
        createAPktTimesTable = "CREATE TABLE APacketTimes(ATime Integer, STime Integer, ETime Integer);"
        cursor.execute(createHPktTimesTable)
        cursor.execute(createAPktTimesTable)

    createHallTable = "CREATE Table Hall(time INTEGER"
    for i in range(0,NumHallSensors):
        createHallTable+="," +"x" + str(i) + " DECIMAL"
    for i in range(0,NumHallSensors):
        createHallTable+="," +"y" + str(i) + " DECIMAL"    
    
    if UseHZ:
        for i in range(0,NumHallSensors):
            createHallTable+="," +"z" + str(i) + " DECIMAL"
        createHallCalNoiseTable = "CREATE TABLE HallCalNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, z DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HallCalBS(BSX DECIMAL, BSY DECIMAL, BSZ DECIMAL);"
    else:
        createHallCalNoiseTable = "CREATE TABLE HallCalNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HallCalBS(BSX DECIMAL, BSY DECIMAL);"
    
    createHallTable += ",Primary KEY(time));"
    
    createAccelTable = "CREATE TABLE Accel(time integer PRIMARY KEY, x DECIMAL, y DECIMAL, z DECIMAL);"
    #print (createHallTable)
    #print (createMetaDataTable)
    #print (createHallCalNoiseTable)
    #print (createHallCalBSTable)
    #print (createAccelTable)
    
    cursor.execute(createMetaDataTable)
    cursor.execute(createHallTable)
    
    cursor.execute(createHallCalNoiseTable)
    cursor.execute(createHallCalBSTable)
    cursor.execute(createAccelTable)
    sqlConn.commit()

def endSession(mySession, offlineClose):
    global lastSessionID
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    lastSessionID = mySession[MDK.ID]
    if offlineClose:
        mySession[MDK.ET] = mySession[MDK.LDT]
    else:
        mySession[MDK.ET] = str(datetime.now())
    
    mySession[MDK.SC] = True
    sqlCmd = "INSERT INTO MetaData(SessionID, IP, StartTime, StopTime, ABS, AST, HBS, HST, HallMASize, HRT, UseHZ, NumSensors, FWVer, BadPacketCount, ESP_START, ESP_STOP) VALUES ("
    
    sqlData = str(mySession[MDK.ID]) +",'" + mySession[MDK.IP][0]+":"+str(mySession[MDK.IP][1]) + "','" + mySession[MDK.ST] +"','"
    sqlData +=  mySession[MDK.ET] +"'," +str(mySession[MDK.ABS]) + "," + str(mySession[MDK.AST])
    sqlData += "," + str(mySession[MDK.HBS]) +"," + str(mySession[MDK.HST]) + ","
    sqlData += str(mySession[MDK.HMA])  + "," +str(mySession[MDK.HRT]) + "," +str(mySession[MDK.HZ])
    sqlData += "," +str(mySession[MDK.NHS]) + ",'" +str(mySession[MDK.FWV])  +"'," +str(mySession[MDK.BPC])+","
    sqlData += str(mySession[MDK.SS])+"," +str(mySession[MDK.ES])
    
    sqlCmd += sqlData + ");"
    cursor.execute(sqlCmd)
    sqlConn.commit()
    sqlConn.close()
    sqlConn = None
    mySession[MDK.SQC] = None
    mySession[MDK.SQP] = None

def InsertHallData(messageData, mySession):
    UseHZ = mySession[MDK.HZ]
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    NumSensors = mySession[MDK.NHS]
    HBSz = mySession[MDK.HBS]
    HRT = mySession[MDK.HRT]
    if DBG_CAP_PKT_TIMES:
        SqlData = (messageData[KEY.HALLTIME],messageData[KEY.REALSTART], messageData[KEY.REALSTOP])
        QueryString = "INSERT INTO HPacketTimes(HTime, STime, ETime) Values(?, ?, ?);"
        cursor.execute(QueryString, SqlData)
    if UseHZ:
        SqlString = "INSERT INTO Hall(time,x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,x5,y5,z5,x6,y6,z6,x7,y7,z7,x8,y8,z8,x9,y9,z9,x10,y10,z10,x11,y11,z11,x12,y12,z12,x13,y13,z13,x14,y14,z14,x15,y15,z15,x16,y16,z16,x17,y17,z17,x18,y18,z18,x19,y19,z19,x20,y20,z20,x21,y21,z21,x22,y22,z22,x23,y23,z23) Values("
    else:
        SqlString = "INSERT INTO Hall(time,x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8,x9,y9,x10,y10,x11,y11,x12,y12,x13,y13,x14,y14,x15,y15,x16,y16,x17,y17,x18,y18,x19,y19,x20,y20,x21,y21,x22,y22,x23,y23) Values("
    currHTime = messageData[KEY.HALLTIME] 
    t=0
    if (UseHZ):
        stepSize = 3
    else:
        stepSize=2
    for i in range(0,HBSz*NumSensors, NumSensors):#outerloop = time
        sqlData = str(t*HRT+currHTime)
        for s in range(0,(NumSensors*stepSize -1),stepSize):#innerloop = sensor
            sqlData += "," + str(messageData[KEY.HALLDATA][s])
            sqlData += "," + str(messageData[KEY.HALLDATA][s+1])
            if (UseHZ):
                sqlData += "," + str(messageData[KEY.HALLDATA][s+2])
        
        sqlData+=")"
        nextSqlString =  SqlString + sqlData
        cursor.execute(nextSqlString)
        sqlConn.commit()
        t+=1

def InsertHallCal(messageData, mySession):
    UseHZ = mySession[MDK.HZ]
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    NumSensors = mySession[MDK.NHS]
    
    if UseHZ:
        SqlString_a = "INSERT INTO HallCalNoise(senseNum, x, y, z) Values( ?, ?, ?, ?);"
        SqlString_b = "INSERT INTO HallCalBS(BSX, BSY, BSZ) Values( ?, ?, ?);"
    else:
        SqlString_a = "INSERT INTO HallCalNoise(senseNum, x, y) Values( ?, ?, ?);"
        SqlString_b = "INSERT INTO HallCalBS(BSX, BSY) Values( ?, ?);"
    for s in range(0,NumSensors):#innerloop = sensor
        if UseHZ:
            sqlData_a = (str(s), messageData[KEY.HCALX][s], messageData[KEY.HCALY][s], messageData[KEY.HCALZ][s])
            
        else:             
            sqlData_a = (str(s), messageData[KEY.HCALX][s], messageData[KEY.HCALY][s])
        cursor.execute(SqlString_a,sqlData_a)
        sqlConn.commit()
    
    if UseHZ:
        sqlData_b = (messageData[KEY.BIOSAVX], messageData[KEY.BIOSAVY], messageData[KEY.BIOSAVZ])
    else:
        sqlData_b = (messageData[KEY.BIOSAVX], messageData[KEY.BIOSAVY])
    cursor.execute(SqlString_b,sqlData_b)
    sqlConn.commit()


def InsertAccelData(messageData, mySession):
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    ABSz = mySession[MDK.ABS]
    AST = mySession[MDK.AST]
    if DBG_CAP_PKT_TIMES:
        SqlData = (messageData[KEY.ACCELTIME],messageData[KEY.REALSTART], messageData[KEY.REALSTOP])
        QueryString = "INSERT INTO APacketTimes(ATime, STime, ETime) Values(?, ?, ?);"
        cursor.execute(QueryString, SqlData)
    
    SqlString = "INSERT INTO Accel(time, x, y, z) Values( ?, ?, ?, ?);"
    currTime = messageData[KEY.ACCELTIME]
    i = 0
    for t in range(0,ABSz):#outerloop = time
        sqlData = (str(t*AST + currTime), messageData["x"][t], messageData["y"][t], messageData["z"][t])
        #pprint.pprint(sqlData)
        cursor.execute(SqlString,sqlData)
        i+=1
        sqlConn.commit()

def shutdownServer():
    global sqlConn, cursor, Sessions
    SaveSettings() #Store last SessionID
    for session in Sessions:
        if (session[MDK.SQP]!=None):#If its a string then no sql db is open
            print("Session #%d not closed correctly! (Closing SQL DB now...)"%session[MDK.ID])    
            session[MDK.SQP].close();
            print("\tTotal Bad Packets recieved: %d" %session[MDK.BPC])
    print("Last Active Session: %d"%sessionID)
    print("Good Bye")
    sysExit()

def clearSettingsAndFiles(quiet):
    global localIP, localPort, bufferSize, sessionID
    if os.path.exists(cfg_fName):
        if not quiet:
            print("RESET\tDeleting old CFG file (%s)" %cfg_fName)
        os.remove(cfg_fName)
        if DBG_CLEAR_EVERY_RUN:
            sessionID = 0
            localIP     = DBG_DEFIP
            localPort   = DBG_DEFPT
            bufferSize  = 20000
            SaveSettings()
        else:
            sessionID = 0
            localIP     = "127.0.0.1"# Localhost for now
            localPort   = 20002
            bufferSize  = 20000
            SaveSettings() 
        directory = os.getcwd() + db_RawDir
        dbDirOk = False
        if os.path.isdir(directory):
            if not quiet:
                print("Session DataBase Directory present")
            dbDirOk = True
        else:
            if not quiet:
                print("Session DataBase Directory Missing. Recreating Now!")
            os.mkdir(directory)
            dbDirOk = True
        if dbDirOk:
            dbFileDirHandle = os.listdir( directory )
            for item in dbFileDirHandle:
                if item.endswith(".db"):
                    os.remove( os.path.join( directory, item ) )
                    if not quiet:
                        print("RESET\tSession DB files Removed")
        else:
                    print("Could not create session DataBase Directory. Check permissions and/or drive space!")

"""
    Main Program Starts here
    
"""
def main():
    #global localIP, localPort, bufferSize, packet, addresses, Sessions
    #global FWVer, HBSz, HT, NumSensors, currHTime, ABSz, AT, UseHZ, sessionID, sessionStartTimeDate
    #global esp32StartMicros, esp32StopMicros, sqlConn, cursor, HUSecLast, AUSecLast, badPacketCount, lastPacketSystemTime
    #global AUSecLast,HUSecLast,activeSessions,sqlConn,cursor, lastSessionID, badPacketCount, HBSz, ABSz
    global localIP, localPort, bufferSize, sessionID
    
    mySession = None
    args = sysArgs
    alen = len(args)
    LoadSettings()
    if alen==5:
        for item in range(1,alen,2):
            if args[item] == "--ip":
                localIP = args[item +1]
            if args[item] == "--port":
                localPort = int(args[item +1])
        SaveSettings()
    elif alen==2:
        if args[1]=="--reset":
                clearSettingsAndFiles(True)
                exit()
    elif alen==1:
                print("Using settings loaded from file")
    else:
        print("Invalid arguments: Specify --ip x.x.x.x & --port nnnnn or --reset")
        sysExit()
    
    if DBG_CLEAR_EVERY_RUN:
        clearSettingsAndFiles(True)
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((localIP, localPort))
    
    print("UDP server up and listening @%s:%d" %(localIP, localPort))
    
    #activeSessions = {"0.0.0.0":-1} # Guard value, not really needed
        
    try:# Try allows us to catch keyboard interrupt
        while(True):      
            bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
            Packet = bytesAddressPair[0]
            address = bytesAddressPair[1]
            unpacked = unpack(Packet)
            #pprint(unpacked)
            sessionMatch = False;
            if unpacked != None:
                mType = getMsgType(unpacked)
                
                #Check Session ID matches mySession as well as update last data packet time stamp (incase device drops out)
                if (mType ==msgType.ACCEL or mType ==msgType.HALL or mType==msgType.CAL_HALL or mType==msgType.END):
                    if (mySession !=None):
                        if (unpacked[KEY.SESSIONID]==mySession[MDK.ID]):
                            mySession[MDK.LDT] = str(datetime.now())
                            sessionMatch = True
                        else:
                            print("Session ID mismatch, discarding!")
                    else:
                        print("No Current Session, but magically we matched session id...")
                
                if (mType==msgType.START):#TODO - Associate SQL conns & cursors to each IP/Session
                    for session in Sessions:
                        if session[MDK.IP][0]==address[0]:
                            print("Device Had Existing Session In Progress\n\tPerforming Offline Close of Session%d"%session[MDK.ID])
                            endSession(session,True)
                            Sessions.remove(session)
                    
                    sessionID +=1#Increment the global ID session counter
                    print("New Session #%d started at %s by %s"%(sessionID,datetime.now(), address[0]),flush=True)
                    
                    SIDresponse = {KEY.SESSIONID:sessionID,KEY.SENDHZ:ServerWantsHZ,KEY.ACCELSAMPLETIME:AST,
                                       KEY.HALLSAMPLETIME:HST,KEY.HALLREPORTTIME:HRT,KEY.HALLMASIZE:HMA,
                                       KEY.SENDSTS:ServerWantsSTS}
                    msgToClient = enc_msgpack(SIDresponse)
                    UDPServerSocket.sendto(msgToClient, address)
                    
                    Sessions.append(sessionObject.copy())
                    mySession = Sessions[-1] #We just added it, so it's the last entry
                    mySession[MDK.ID] = sessionID
                    mySession[MDK.IP] = address
                    mySession[MDK.ST] = str(datetime.now())
                    mySession[MDK.ABS] = unpacked[KEY.ACCELBLOCKSIZE]
                    mySession[MDK.AST] = AST
                    mySession[MDK.HBS] = unpacked[KEY.HALLBLOCKSIZE]
                    mySession[MDK.HST] = HST
                    mySession[MDK.HMA] = HMA
                    mySession[MDK.HRT] = HRT
                    mySession[MDK.HZ] = (unpacked[KEY.HALLHASZ]==1)&(ServerWantsHZ==1)
                    mySession[MDK.NHS] = unpacked[KEY.NUMHALLSENSORS]
                    mySession[MDK.FWV] = unpacked[KEY.FWVER]
                    mySession[MDK.SS] = unpacked[KEY.STARTSESSION]
                    newSQLdb(mySession)
                    #print(mySession)
    
                elif ((mType==msgType.END) and sessionMatch):
                    mySession[MDK.ES] = unpacked[KEY.ENDSESSION]
                    endSession(mySession,False)
                    byeResponse = {KEY.SRVOK: mySession[MDK.LHS]} #Send time stamp of first entry of last packet (probably wont be used)
                    byeBytes =enc_msgpack(byeResponse)
                    UDPServerSocket.sendto(byeBytes, mySession[MDK.IP])
                    print("Session #%d closed!" %(mySession[MDK.ID]), flush=True)
                    #markSessionComplete(lastSessionID)  #Probably just going to go off of file open or not!
                    #print(mySession)
                    Sessions.remove(mySession)
                    mySession = None
                    
                elif (mType==msgType.CHECK):
                    chkResponse = {KEY.SRVOK: 1}
                    chkBytes =enc_msgpack(chkResponse)
                    UDPServerSocket.sendto(chkBytes, address)
                    print("Device @%s checked server alive!" %address[0])
                
                elif ((mType==msgType.HALL) and sessionMatch):
                    if (ServerWantsSTS):
                        print("H Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end = "")
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Hdiff = unpacked[KEY.REALSTART]-mySession[MDK.LHS]
                        print("    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed),end = "")
                        if mySession[MDK.LHS]>=0:
                                print("    (Start Delta=%d)"%Hdiff)
                        else:
                            print("", flush=True)
                        mySession[MDK.LHS] = unpacked[KEY.REALSTART]
                    else:
                        print("H Data Rx'd (%d Bytes)"%Packet.__sizeof__(), flush=True)
                    #pprint(unpacked)
                    InsertHallData(unpacked, mySession)
    
                elif ((mType==msgType.ACCEL) and sessionMatch):
                    if (ServerWantsSTS):
                        print("A Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end ="")
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Adiff = unpacked[KEY.REALSTART]-mySession[MDK.LAS]
                        print("    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed),end = "")
                        if mySession[MDK.LAS]>=0:
                                print("    (Start Delta=%d)"%Adiff)
                        else:
                            print("", flush=True)
                        mySession[MDK.LAS] = unpacked[KEY.REALSTART]
                    else:
                        print("A Data Rx'd (%d Bytes)"%Packet.__sizeof__(), flush=True)
                    #pprint(unpacked)
                    #print("\n\n",flush=True)
                    InsertAccelData(unpacked, mySession)
                    
                elif ((mType==msgType.CAL_HALL) and sessionMatch):
                    InsertHallCal(unpacked, mySession)
                    print("Cal Data Rx'd (%d Bytes)"%Packet.__sizeof__())
                else:
                    print("Bad Packet Rx'd from: %s" %address[0])
                    if mySession !=None:
                        mySession[MDK.BPC]+=1
                    #maybe send some data...or not
                    #perhaps implement some sort of flag to check if the next packet is the time step after
                    #then just auto fill gap with interpolated values...or fix it in post(processing)
                    #UDPServerSocket.sendto(bytesToSend, address)
        print("Shutting down")
        shutdownServer()
        
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt - Shutting Down", flush=True)
        shutdownServer()

if __name__ == "__main__":
    main()