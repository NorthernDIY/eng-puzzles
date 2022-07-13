#!/bin/python3
#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, sqlite3,os
import AMazeThing_Keys as KEY
import AMazeThingSessionFormat as MDK
from AMazeThingSessionFormat import SessionObject
from AMazeThingSessionFormat import DBSessionObjectTags as metaTags
from AMazeThingSessionFormat import DBSessionObjectTagTypes as metaTagTypes
from AMazeThingSessionFormat import KPT_HTAGS, KPT_HTAGTYPES, KPT_ATAGS, KPT_ATAGTYPES
from msgpack import loads as dec_msgpack
from msgpack import dumps as enc_msgpack
from sys import argv as sysArgs
from sys import exit as sysExit
from datetime import datetime
import threading, signal
import time
import netifaces
from enum import Enum
from pprint import pprint
import random
import json

import queue
#packet = None
#addresses= None

#Server Properties
VERSIONNUM = 0.85
VERSIONSTRING = "AMazeThing UDP Server    (v" + str(VERSIONNUM) + ")"

#Service broadcaster Parameters
SERVICETXT = "MAZE_SERVER"
BCAST_PORT = 1998 #UDP Broadcast port to announce server availability (SERVER -> ESP32)
BCAST_INT = 5
BCAST_THREAD = None

#Session Handler Parameters
START_PORT = 1999 #UDP Port for negotiating a new session (SERVER <-> ESP32)
START_THREAD = None

KB_THREAD = None #Thread that watches KB input to toggle server settings and quit

specificIP = "0.0.0.0"
#Things you might want to change
#Network Settings (Defaults)
localIP_d     = "127.0.0.1"# Localhost for now
localPort_d   = 20002
bufferSize_d  = 20000

portMin = 12000
portMax = 15000

finishedSessions = [None,]
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


DBG_CLEAR_EVERY_RUN = 1
DBG_DEFIP = "192.168.137.1"
#DBG_DEFIP = "10.42.0.1"
DBG_DEFPT = 1999
DBG_CAP_PKT_TIMES =1


#Network setting variables
localIP     = DBG_DEFIP#localIP_d
bufferSize  = bufferSize_d

usedPorts = [BCAST_PORT,START_PORT,]



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

#Runtime State Variables, things switched on/off by input monitoring thread
printPacketReciepts = True
timeToGo = False


#Network specific Utility functions
def getIFIPlist():#Works better on linux than other options...
    #https://www.delftstack.com/howto/python/get-ip-address-python/
    interfacesList = netifaces.interfaces()
    for ifaceName in interfacesList:
        addresses = [i['addr'] for i in netifaces.ifaddresses(ifaceName).setdefault(netifaces.AF_INET, [{'addr':'No IP addr'}] )]
    print(addresses, flush=True)
    return addresses


#handle keyboard input to properly shutdown program in windows.....
#based on https://stackoverflow.com/a/57387909
class KB_Watcher(threading.Thread):

    def __init__(self, input_cbk = None, name='KB Watch Thread'):
        self.input_cbk = input_cbk
        super(KB_Watcher, self).__init__(name=name)
        self.shutdown_flag = threading.Event()

    #def run(self):
    def run(self):
        blankInputs = 0
        print(VERSIONSTRING)
        print("\tq <enter> to Quit")
        print("\tv <enter> to Enable/Disable Incoming Packet Messages")
        while (not self.shutdown_flag.is_set()):
            try:
                inputC = input() #waits to get input + Return
                my_callback(inputC)
            except EOFError:
                blankInputs+=1
        print("KB Watch Stopped")

def my_callback(inp):
    global timeToGo, printPacketReciepts
    #evaluate the keyboard input
    if inp=='q' or inp == "Q":
        timeToGo = True
        print("Shutting down Server...")
    if inp=='v' or inp == "V":
        if (not printPacketReciepts):
            print("Enabling packet Reciept Messages")
            printPacketReciepts = True
        else:
            print("Disabling packet Reciept Messages")
            printPacketReciepts = False
#End of code from SOF


class SERVER_NewSessionHandler(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.shutdown_flag = threading.Event()
        self.Sessions = []
        self.aSessionIsComplete = threading.Event()
        
    
    def run(self):
        global sessionID,timeToGo,finishedSessions, usedPorts
        time.sleep(2)
        self.shutdown_flag = threading.Event()
        print("Session Listener Started @ Port:",START_PORT," (",self.ident,")")
        UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        UDPServerSocket.bind((localIP, START_PORT))
        UDPServerSocket.settimeout(1)

        while not self.shutdown_flag.is_set():
            try:# Try allows us to catch keyboard interrupt
                quietCount = 0
                havePacket = False
                unpacked = None
                try:
                    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
                    #print("got packet")
                    havePacket = True
                except:
                    quietCount +=1
                if havePacket:
                    Packet = bytesAddressPair[0]
                    address = bytesAddressPair[1]
                    unpacked = unpack(Packet)
                if (unpacked != None):
                    mType = getMsgType(unpacked)
                    if (mType==msgType.START):#TODO - Associate SQL conns & cursors to each IP/Session
                        for session in self.Sessions:
                            if session != None:
                                if session[MDK.IP][0]==address[0]:
                                    print("Device Had Existing Session In Progress (#%d)"%session[MDK.ID])
                                    session[MDK.SRV_THREAD].orphaned.set()
                                    session[MDK.SRV_THREAD].join()
                                    self.Sessions.remove(session)
                        sessionID +=1#Increment the global ID session counter
                        self.Sessions.append(SessionObject.copy())
                        mySession = self.Sessions[-1] #We just added it, so it's the last entry
                        mySession[MDK.ID] = sessionID
                        mySession[MDK.IP] = address[0]
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
                        mySession[MDK.DBG_SS] = unpacked[KEY.STARTSESSION]
                        aPort = pickRandomPort()
                        #print("\tPicked Port:", aPort)
                        mySession[MDK.PT] = aPort
                        ServerThread = SERVER_SessionHandler(mySession)
                        usedPorts.append(aPort)
                        #print(mySession)
                        #print(id(mySession))
                        
                        mySession[MDK.SRV_THREAD] = ServerThread
                        mySession[MDK.SRV_THREAD].start()
                    elif (mType==msgType.CHECK):
                        chkResponse = {KEY.SRVOK: 1}
                        chkBytes =enc_msgpack(chkResponse)
                        UDPServerSocket.sendto(chkBytes, address)
                        #print("Device @%s checked server alive!" %address[0])

            except KeyboardInterrupt:
                print("New Session Handler: KB Int")
                timeToGo = True
                self.shutdown_flag.set()
            
            #Normally when a session appears in finishedSessions, we can remove
            #it from our list of sessions and mark the port available again
            if self.aSessionIsComplete.is_set():
                while len(finishedSessions)>1:
                    session = finishedSessions[-1]
                    if session !=None:
                        usedPorts.remove(session[MDK.PT])#The port is no longer in use so it can go back into the pool
                        session[MDK.SRV_THREAD].join()#Make sure the thread is done executing
                        self.Sessions.remove(session)
                        finishedSessions.remove(session)
                self.aSessionIsComplete.clear()
                    
        #(from while waaay above)shutdownflag is now set so we are quitting soon!
        for session in self.Sessions:
            print("SHUTDOWN: Closing active sessions")
            print(self.Sessions)
            if session!=None:
                session[MDK.SRV_THREAD].shutdown_flag.set()
                time.sleep(1)
                session[MDK.SRV_THREAD].join()
                
                
class SERVER_SessionHandler(threading.Thread):
    
    def __init__(self, session):
        threading.Thread.__init__(self)
        self.shutdown_flag = threading.Event()
        self.mySession = session
        self.orphaned = threading.Event()
        
    def run(self):
        global finishedSessions
        mySession = self.mySession
        sessionNum = mySession[MDK.ID]
        PORT = mySession[MDK.PT]
        IP = mySession[MDK.IP]
        address = (IP,PORT)
        addressStart = (IP,START_PORT)
        UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        UDPServerSocket.bind((localIP, PORT))
        UDPServerSocket.settimeout(1)
        print("New Session #%d started at %s by %s:%d"%(sessionNum,datetime.now(), address[0],address[1]),flush=True)
        SIDresponse = {KEY.SESSIONID:sessionNum,
                       KEY.SENDHZ:ServerWantsHZ,
                       KEY.ACCELSAMPLETIME:AST,
                       KEY.HALLSAMPLETIME:HST,
                       KEY.HALLREPORTTIME:HRT,
                       KEY.HALLMASIZE:HMA,
                       KEY.SENDSTS:ServerWantsSTS,
                       KEY.SESSIONPORT:mySession[MDK.PT]}
        msgToClient = enc_msgpack(SIDresponse)
        UDPServerSocket.sendto(msgToClient, addressStart)
        newSQLdb(mySession)
        while((not self.shutdown_flag.is_set()) and (not self.orphaned.is_set())):
            unpacked = None
            bytesAddressPair = None
            havePacket = False
            sessionMatch = False
            #JULY 12 END OF DAY
            """
            Todo:
            Confirm that session can be ended when orphan set.
            Confirm that packets arent lost if timeout occurs
            Clean up exit signalling code
            FIX IP SAVE IN METADATA
            Re-integrate reset/command line arguments stuff
            Expand saved parameters to include specific ports, session port ranges + extra settings
            """
            try:
                bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
                havePacket = True
            except:
                havePacket = False
            if havePacket:
                Packet = bytesAddressPair[0]
                address = bytesAddressPair[1]
                unpacked = unpack(Packet)
                if unpacked != None:
                    sessionMatch = (mySession[MDK.ID] == unpacked[KEY.SESSIONID])
            if ((unpacked != None) & sessionMatch):
                mType = getMsgType(unpacked)
                
                if ((mType==msgType.HALL)):
                    if (ServerWantsSTS):
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Hdiff = unpacked[KEY.REALSTART]- mySession[MDK.LHS]
                        
                    InsertHallData(unpacked, mySession)
                    if printPacketReciepts:
                            message = "H Data Rx'd (%d Bytes)"%Packet.__sizeof__()
                            message+= "    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed) if ServerWantsSTS else ''
                            message+= "    (Start Delta=%d)"%Hdiff if mySession[MDK.LHS]>=0 else ""
                            print(message, flush = True)
                    mySession[MDK.LHS] = unpacked[KEY.REALSTART]
        
                elif ((mType==msgType.ACCEL)):
                    if (ServerWantsSTS):
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Adiff = unpacked[KEY.REALSTART]-mySession[MDK.LAS]
                
                    InsertAccelData(unpacked, mySession)
                    if printPacketReciepts:
                        message = "A Data Rx'd (%d Bytes)"%Packet.__sizeof__()
                        message+= "    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed) if ServerWantsSTS else ''
                        message+= "    (Start Delta=%d)"%Adiff if mySession[MDK.LAS]>=0 else ""
                        print(message, flush = True)
                    
                    mySession[MDK.LAS] = unpacked[KEY.REALSTART]
                    
                elif ((mType==msgType.CAL_HALL)):
                    InsertHallCal(unpacked, mySession)
                    if printPacketReciepts:
                        print("Cal Data Rx'd (%d Bytes)"%Packet.__sizeof__())
                elif ((mType==msgType.END)):
                    mySession[MDK.DBG_SE] = unpacked[KEY.ENDSESSION]
                    
                    byeResponse = {KEY.SRVOK: mySession[MDK.LHS]} #Send time stamp of first entry of last packet (probably wont be used)
                    byeBytes =enc_msgpack(byeResponse)
                    UDPServerSocket.sendto(byeBytes, (mySession[MDK.IP],mySession[MDK.PT]))
                    
                    self.shutdown_flag.set()
                else:
                    if printPacketReciepts:
                        print("Bad Packet Rx'd from: %s" %address[0])
                        print(Packet)
                        print(unpacked)
                    if mySession !=None:
                        mySession[MDK.BPC]+=1
        print("Thread ending?")
        if self.orphaned.is_set():
            print("Session #%d closed! (Orphaned)" %(mySession[MDK.ID]),flush = True)
            endSession(mySession,True)
            print("Orphaned")
        else:
            print("Session #%d closed! (Completed)" %(mySession[MDK.ID]), flush=True)
            endSession(mySession,False)
            print("Shutdown")
        
        finishedSessions.append(mySession)
        START_THREAD.aSessionIsComplete.set()#Notify session initialization thread that we are done

class ServiceBroadcaster(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.shutdown_flag = threading.Event()
    
    def run(self):
        ServiceDictionary = {"SVC":SERVICETXT,"VER":VERSIONNUM,"PORT":START_PORT}
        ServiceMessage = enc_msgpack(ServiceDictionary,use_single_float=True,strict_types=True)
        interfaces = {"127.0.0.2", localIP}#localhost, .2 since windows prohibits .1
        while not self.shutdown_flag.is_set():
            for ip in interfaces:
                UDPBROADCASTSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
                UDPBROADCASTSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                UDPBROADCASTSocket.bind((ip,0))
                UDPBROADCASTSocket.sendto(ServiceMessage,("255.255.255.255", BCAST_PORT))
                UDPBROADCASTSocket.close()
            time.sleep(BCAST_INT)

############################################

def sigTerm_Handler(_signo, _stack_frame):
    if BCAST_THREAD != None:
        BCAST_THREAD.shutdown_flag.set()
    if START_THREAD != None:
        START_THREAD.shutdown_flag.set()
    if KB_THREAD != None:
        KB_THREAD.shutdown_flag.set()
    print("SigTerm Exit Handled Gracefully")
    
class msgType(Enum):
    JUNK= 0
    START =1
    END = 2
    HALL = 3
    ACCEL = 4
    CAL_HALL = 5
    CHECK = 6


def pickRandomPort():
    global usedPorts
    aPort = random.randint(portMin, portMax)
    while(isPortUsed(aPort)):
        aPort = random.randint(portMin, portMax)
        #print("Next RND Port")
    #print(usedPorts)
    return aPort

def isPortUsed(checkport):
    inList = False
    for port in usedPorts:
        if port == checkport:
                inList = True
                break
    return inList

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
    cfgVars = {"IP": localIP, "Port": START_PORT, "bSize": bufferSize, "SID":sessionID}
    with open(cfg_fName, 'w') as outfile:
        json.dump(cfgVars, outfile)

def LoadSettings():
    global sessionID, localIP, START_PORT, bufferSize
    if os.path.exists(cfg_fName):
        with open(cfg_fName) as json_file:
            cfgVars = json.load(json_file)
        sessionID = cfgVars["SID"]
        localIP = cfgVars["IP"]
        START_PORT = cfgVars["Port"]
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
    
    #Create Table for metadata
    createMetaDataTable = "CREATE TABLE MetaData("
    for tagPos in range(0, len(metaTags)):
        if tagPos!=0:
            createMetaDataTable +=", "
        createMetaDataTable += metaTags[tagPos] + " " + metaTagTypes[tagPos]
    createMetaDataTable += ");"
    
    #Create Table for Hall Data, Also create the Sensor Noise Tables and BIOSAVART RT Constant tables        
    createHallTable = "CREATE Table Hall(time INTEGER"
    for i in range(0,NumHallSensors):
        createHallTable+="," +"x" + str(i) + " DECIMAL"
    for i in range(0,NumHallSensors):
        createHallTable+="," +"y" + str(i) + " DECIMAL"    
    
    if UseHZ:
        for i in range(0,NumHallSensors):
            createHallTable+="," +"z" + str(i) + " DECIMAL"
        createHallCalNoiseTable = "CREATE TABLE HNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, z DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HBioSav(BSX DECIMAL, BSY DECIMAL, BSZ DECIMAL);"
    else:
        createHallCalNoiseTable = "CREATE TABLE HNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HBioSav(BSX DECIMAL, BSY DECIMAL);"
    
    createHallTable += ",Primary KEY(time));"
    
    #Create Table for Accelerometer Data
    createAccelTable = "CREATE TABLE Accel(time INTEGER PRIMARY KEY, x DECIMAL, y DECIMAL, z DECIMAL);"
    
    cursor.execute(createMetaDataTable)
    cursor.execute(createHallTable)
    cursor.execute(createHallCalNoiseTable)
    cursor.execute(createHallCalBSTable)
    cursor.execute(createAccelTable)
    
    #Create the table of packet times if we are interested in that at all...
    if DBG_CAP_PKT_TIMES:
        createHPktTimesTable = "CREATE TABLE HPacketTimes("
        createAPktTimesTable = "CREATE TABLE APacketTimes("
        for tagPos in range(0, len(KPT_HTAGS)):
            if tagPos!=0:
                createAPktTimesTable +=", "
                createHPktTimesTable +=", "
            createHPktTimesTable += KPT_HTAGS[tagPos] +" " + KPT_HTAGTYPES[tagPos]
            createAPktTimesTable += KPT_ATAGS[tagPos] +" " + KPT_ATAGTYPES[tagPos]

        createHPktTimesTable += ");"
        createAPktTimesTable += ");"
        cursor.execute(createHPktTimesTable)
        cursor.execute(createAPktTimesTable)
    
    sqlConn.commit()

def endSession(mySession, offlineClose):
    global lastSessionID
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    lastSessionID = mySession[MDK.ID]
    mySession[MDK.SC] = 1
    mySession[MDK.DBG_KPT] = DBG_CAP_PKT_TIMES
    if offlineClose:
        mySession[MDK.ET] = mySession[MDK.LDT]
    else:
        mySession[MDK.ET] = str(datetime.now())
    
    sqlCmdp1 = "Insert Into MetaData("
    sqlCmdp2 = ") Values("
    sqlData = ""
    for tagPos in range(0, len(metaTags)):
        if tagPos!=0:
            sqlCmdp1 +=", "
            sqlData +=", "
        sqlCmdp1 += metaTags[tagPos]
        if metaTagTypes[tagPos] == "TEXT":
            sqlData+="'"
        if metaTags[tagPos] == MDK.IP:
            sqlData += str(mySession[metaTags[tagPos]][0]) + ":" +str(mySession[metaTags[tagPos]][1])
        else:
            sqlData += str(mySession[metaTags[tagPos]])
        if metaTagTypes[tagPos] == "TEXT":
            sqlData+="'"
    
    sqlCmd = sqlCmdp1 + sqlCmdp2 + sqlData +")"
    #print(sqlCmd)
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
    NHS = mySession[MDK.NHS]
    HBSz = mySession[MDK.HBS]
    HRT = mySession[MDK.HRT]
    if DBG_CAP_PKT_TIMES:
        SqlData = (messageData[KEY.HALLTIME],messageData[KEY.REALSTART], messageData[KEY.REALSTOP])
        DBGQueryString = "INSERT INTO HPacketTimes(HTime, Start, End) Values(?, ?, ?);"
        cursor.execute(DBGQueryString, SqlData)
    
    hallQueryString = "INSERT INTO Hall(time"
    for i in range(0,NHS):
        hallQueryString+="," +"x" + str(i) + ",y"+ str(i)
        if UseHZ:
            hallQueryString+="," +"z" + str(i)
    hallQueryString +=") Values("
    currHTime = messageData[KEY.HALLTIME] 
    t=0
    if (UseHZ):
        stepSize = 3
    else:
        stepSize=2
    for i in range(0,HBSz*NHS, NHS):#outerloop = time
        hallData = str(t*HRT+currHTime)
        for s in range(0,(NHS*stepSize -1),stepSize):#innerloop = sensor
            hallData += "," + str(messageData[KEY.HALLDATA][s])#x
            hallData += "," + str(messageData[KEY.HALLDATA][s+1])#y
            if (UseHZ):
                hallData += "," + str(messageData[KEY.HALLDATA][s+2])#z
        nextSqlString =  hallQueryString + hallData + ")"
        cursor.execute(nextSqlString)
        sqlConn.commit()
        t+=1

def InsertHallCal(messageData, mySession):
    UseHZ = mySession[MDK.HZ]
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    NHS = mySession[MDK.NHS]
    
    if UseHZ:
        SqlString_a = "INSERT INTO HNoise(senseNum, x, y, z) Values( ?, ?, ?, ?);"
        SqlString_b = "INSERT INTO HBioSav(BSX, BSY, BSZ) Values( ?, ?, ?);"
    else:
        SqlString_a = "INSERT INTO HNoise(senseNum, x, y) Values( ?, ?, ?);"
        SqlString_b = "INSERT INTO HBioSav(BSX, BSY) Values( ?, ?);"
    for s in range(0,NHS):#innerloop = sensor
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
        QueryString = "INSERT INTO APacketTimes(ATime, Start, End) Values(?, ?, ?);"
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
            endSession(session,True)#Perform offline close
            #session[MDK.SQP].close();
            print("\tTotal Bad Packets recieved: %d" %session[MDK.BPC])
    print("Last Active Session: %d"%sessionID)
    print("Good Bye")
    sysExit()

def clearSettingsAndFiles(quiet):
    global localIP, START_PORT, bufferSize, sessionID
    if os.path.exists(cfg_fName):
        if not quiet:
            print("RESET\tDeleting old CFG file (%s)" %cfg_fName)
        os.remove(cfg_fName)
        if DBG_CLEAR_EVERY_RUN:
            sessionID = 0
            localIP     = DBG_DEFIP
            START_PORT   = DBG_DEFPT
            bufferSize  = 20000
            SaveSettings()
        else:
            sessionID = 0
            localIP     = "127.0.0.1"# Localhost for now
            START_PORT   = 20002
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
    global localIP, START_PORT, bufferSize, sessionID,timeToGo
        
    if DBG_CLEAR_EVERY_RUN:
        clearSettingsAndFiles(True)
    KB_THREAD = KB_Watcher()
    START_THREAD = SERVER_NewSessionHandler()
    BCAST_THREAD = ServiceBroadcaster()
    
    KB_THREAD.daemon = True
    #START_THREAD.daemon = True
    #BCAST_THREAD.daemon = True
    
    KB_THREAD.start()
    START_THREAD.start()
    BCAST_THREAD.start()
    
    while(not timeToGo):
        time.sleep(0.5)
    START_THREAD.shutdown_flag.set()
    BCAST_THREAD.shutdown_flag.set()
    KB_THREAD.shutdown_flag.set()
    sysExit() 


    args = sysArgs
    alen = len(args)
    LoadSettings()
    if alen==5:
        for item in range(1,alen,2):
            if args[item] == "--ip":
                localIP = args[item +1]
            if args[item] == "--port":
                START_PORT = int(args[item +1])
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

    
    """
    #ListenerThread = threading.Thread(target = SERVER_listenForHello)
    #ListenerThread.daemon = True
    #ListenerThread.start()
    print("wtf")
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((localIP, START_PORT))
    print("UDP server up and listening @%s:%d" %(localIP, START_PORT))
    
    UDPServerSocket.settimeout(1);
    try:# Try allows us to catch keyboard interrupt
        quietCount = 0
        while(not timeToGo):
            havePacket = False
            sessionMatch = False
            unpacked = None
            try:
                bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
                havePacket = True
            except:
                quietCount +=1
            if havePacket:
                Packet = bytesAddressPair[0]
                address = bytesAddressPair[1]
                unpacked = unpack(Packet)
                
            if (unpacked != None):
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
                            print("Device Had Existing Session In Progress\n\tPerforming Offline Close of Session #%d"%session[MDK.ID])
                            endSession(session,True)
                            Sessions.remove(session)
                    
                    sessionID +=1#Increment the global ID session counter
                    print("New Session #%d started at %s by %s"%(sessionID,datetime.now(), address[0]),flush=True)
                    
                    SIDresponse = {KEY.SESSIONID:sessionID,KEY.SENDHZ:ServerWantsHZ,KEY.ACCELSAMPLETIME:AST,
                                       KEY.HALLSAMPLETIME:HST,KEY.HALLREPORTTIME:HRT,KEY.HALLMASIZE:HMA,
                                       KEY.SENDSTS:ServerWantsSTS,KEY.SESSIONPORT:mySession[MDK.PORT]}
                    msgToClient = enc_msgpack(SIDresponse)
                    UDPServerSocket.sendto(msgToClient, address)
                    
                    Sessions.append(SessionObject.copy())
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
                    mySession[MDK.DBG_SS] = unpacked[KEY.STARTSESSION]
                    newSQLdb(mySession)
                    #print(mySession)
    
                elif ((mType==msgType.END) and sessionMatch):
                    mySession[MDK.DBG_SE] = unpacked[KEY.ENDSESSION]
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
                    if printPacketReciepts:
                        print("Device @%s checked server alive!" %address[0])
                
                elif ((mType==msgType.HALL) and sessionMatch):
                    if (ServerWantsSTS):
                        if printPacketReciepts:
                            print("H Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end = "")
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Hdiff = unpacked[KEY.REALSTART]-mySession[MDK.LHS]
                        if printPacketReciepts:
                            print("    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed),end = "")
                            if mySession[MDK.LHS]>=0:
                                print("    (Start Delta=%d)"%Hdiff)
                            else:
                                print("", flush=True)
                        mySession[MDK.LHS] = unpacked[KEY.REALSTART]
                    elif printPacketReciepts:
                        print("H Data Rx'd (%d Bytes)"%Packet.__sizeof__(), flush=True)
                    #pprint(unpacked)
                    InsertHallData(unpacked, mySession)
    
                elif ((mType==msgType.ACCEL) and sessionMatch):
                    if (ServerWantsSTS):
                        if printPacketReciepts:
                            print("A Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end ="")
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Adiff = unpacked[KEY.REALSTART]-mySession[MDK.LAS]
                        if printPacketReciepts:
                            print("    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed),end = "")
                            if mySession[MDK.LAS]>=0:
                                print("    (Start Delta=%d)"%Adiff)
                            else:
                                print("", flush=True)
                        mySession[MDK.LAS] = unpacked[KEY.REALSTART]
                    elif printPacketReciepts:
                        print("A Data Rx'd (%d Bytes)"%Packet.__sizeof__(), flush=True)
                    #pprint(unpacked)
                    #print("\n\n",flush=True)
                    InsertAccelData(unpacked, mySession)
                    
                elif ((mType==msgType.CAL_HALL) and sessionMatch):
                    InsertHallCal(unpacked, mySession)
                    if printPacketReciepts:
                        print("Cal Data Rx'd (%d Bytes)"%Packet.__sizeof__())
                else:
                    if printPacketReciepts:
                        print("Bad Packet Rx'd from: %s" %address[0])
                    if mySession !=None:
                        mySession[MDK.BPC]+=1
                    #maybe send some data...or not
                    #perhaps implement some sort of flag to check if the next packet is the time step after
                    #then just auto fill gap with interpolated values...or fix it in post(processing)
                    #UDPServerSocket.sendto(bytesToSend, address)
        #kthread.join(4)
        shutdownServer()
        
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt - Shutting Down", flush=True)
        shutdownServer()
"""
if __name__ == "__main__":
    main()
    

