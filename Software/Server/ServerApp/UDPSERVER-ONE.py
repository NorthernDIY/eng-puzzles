#!/bin/python3
#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, sqlite3,os
import AMazeThing_Keys as KEY
import AMazeThingSessionFormat as MDK
import UDPSERVER_CFG as CFG
from UDPSERVER_CFG import _DEFAULTS
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

#import queue

SVR_CFG = _DEFAULTS.copy()


#Server Properties
VERSIONNUM = 0.85
VERSIONSTRING = "AMazeThing UDP Server    (v" + str(VERSIONNUM) + ")"

##########################
#Service broadcaster Variables
SERVICETXT = "MAZE_SERVER"

BCAST_THREAD = None         #Service Broadcast Thread Handle, kept so we can signal shutdown etc.

##########################
#Session Handler Variables
START_THREAD = None         #Service Initiator Thread Handle, kept so we can signal shutdown etc.  Sessions, also set flags as they complete

usedPorts = [SVR_CFG[CFG.K_BCP],    #Active session ports are appended to this list on start, and removed on finish
             SVR_CFG[CFG.K_STP],]

finishedSessions = [None,]  #List of sessions that are complete (To be Used for launching Processing threads)


#(Do Not Change! Edit config file or use the command line arguments to adjust them!
#Client Device parameters (Sent at session start to ESP32, from session specific 


#FileName Things
DB_FNAME = './' + SVR_CFG[CFG.K_DBRD] + SVR_CFG[CFG.K_SDPFX] #Session db prefix
CFG_FNAME = 'ServerConfig.json' #Filename of settings storage

#Keyboard handler Stuff
KB_THREAD = None            #Thread that watches KB input to toggle server settings and quit gracefully

#Run Time State Variable
timeToGo = False    #Set to True = shut things down and exit gracefully

"""
Todo:	-(COMPLETE JULY 4/2022)     Fix missing RawDat directory error
		-Detect current OS vs Last OS at startup and autoreset if different (Helpful for development)
		-Watch Session ID of incoming packet and discard if not associated with IP (Basic anti-spoofing)
		-Figure out how to close out sessions after some time of no activity --or- when next session started by same IP
		-Netifaces get all interfaces, no joke stop skipping things randomly	
"""
"""
#Network specific Utility functions
def getIFIPlist():#Works better on linux than other options...
    #https://www.delftstack.com/howto/python/get-ip-address-python/
    interfacesList = netifaces.interfaces()
    for ifaceName in interfacesList:
        addresses = [i['addr'] for i in netifaces.ifaddresses(ifaceName).setdefault(netifaces.AF_INET, [{'addr':'No IP addr'}] )]
    print(addresses, flush=True)
    return addresses
"""

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
    global timeToGo, DISP_PKT_RX
    #evaluate the keyboard input
    if inp=='q' or inp == "Q":
        timeToGo = True
        print("Shutting down Server...")
    if inp=='v' or inp == "V":
        if (not DISP_PKT_RX):
            print("Enabling packet Reciept Messages")
            DISP_PKT_RX = True
        else:
            print("Disabling packet Reciept Messages")
            DISP_PKT_RX = False
#End of code from SOF


class SERVER_NewSessionHandler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.shutdown_flag = threading.Event()
        self.Sessions = []
        self.aSessionIsComplete = threading.Event()
    
    def run(self):
        global SESSIONID,timeToGo,finishedSessions, usedPorts
        time.sleep(2)
        self.shutdown_flag = threading.Event()
        print("Session Listener Started @ Port:",START_PORT," (",self.ident,")")
        UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        UDPServerSocket.bind((LOCALIP, START_PORT))
        UDPServerSocket.settimeout(1)

        while not self.shutdown_flag.is_set():
            try:# Try allows us to catch keyboard interrupt (or tries to...)
                quietCount = 0
                havePacket = False
                unpacked = None
                try:
                    bytesAddressPair = UDPServerSocket.recvfrom(BUFFERSIZE)
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
                        SVR_CFG[CFG.K_SID] +=1#Increment the global ID session counter
                        aPort = pickRandomPort()
                        usedPorts.append(aPort)
                        
                        self.Sessions.append(SessionObject.copy())
                        mySession = self.Sessions[-1] #We just added it, so it's the last entry
                        ServerThread = SERVER_SessionHandler(mySession)
                        
                        mySession[MDK.SRV_THREAD] = ServerThread
                        mySession[MDK.ID] = SVR_CFG[CFG.K_SID]
                        mySession[MDK.IP] = address[0]
                        mySession[MDK.PT] = aPort
                        mySession[MDK.ST] = str(datetime.now())
                        mySession[MDK.ABS] = unpacked[KEY.ACCELBLOCKSIZE]
                        mySession[MDK.AST] = SVR_CFG[CFG.K_AST]
                        mySession[MDK.HBS] = unpacked[KEY.HALLBLOCKSIZE]
                        mySession[MDK.HST] = SVR_CFG[CFG.K_HST]
                        mySession[MDK.HMA] = SVR_CFG[CFG.K_HMA]
                        mySession[MDK.HRT] = SVR_CFG[CFG.K_HRT]
                        mySession[MDK.HZ] = (unpacked[KEY.HALLHASZ]==1)&(SVR_CFG[CFG.K_GHZ]==1)
                        mySession[MDK.NHS] = unpacked[KEY.NUMHALLSENSORS]
                        mySession[MDK.FWV] = unpacked[KEY.FWVER]
                        mySession[MDK.TIME_SS] = unpacked[KEY.STARTSESSION]#ESP32 ms since Power on
                        mySession[MDK.TIME_STS] = SVR_CFG[CFG.K_GST]
                        mySession[MDK.TIME_KPT] = (SVR_CFG[CFG.K_GST]==1 & SVR_CFG[CFG.K_KPT]==1)

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
        myIP = mySession[MDK.IP]
        myPort = mySession[MDK.PT]
        address = (myIP,PORT)
        addressStart = (myIP,START_PORT)
        myHZ = mySession[MDK.HZ]
        myAST = mySession[MDK.AST]
        myHST = mySession[MDK.HST]
        myHRT = mySession[MDK.HRT]
        myHMA = mySession[MDK.HMA]
        mySTS = mySession[MDK.TIME_STS]
        
        UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        UDPServerSocket.bind((LOCALIP, myPort)) 
        UDPServerSocket.settimeout(1)
        #Note: Should migrate from sending Global Variable Values to values set in session object
        print("New Session #%d started at %s by %s:%d"%(sessionNum,datetime.now(), address[0],address[1]),flush=True)
        SessionStartResponse = {KEY.SESSIONID:sessionNum,
                       KEY.SENDHZ:myHZ,
                       KEY.ACCELSAMPLETIME:myAST,
                       KEY.HALLSAMPLETIME:myHST,
                       KEY.HALLREPORTTIME:myHRT,
                       KEY.HALLMASIZE:myHMA,
                       KEY.SENDSTS:mySTS,
                       KEY.SESSIONPORT:myPort}
        msgToClient = enc_msgpack(SessionStartResponse)
        UDPServerSocket.sendto(msgToClient, addressStart)#Address Start, pretend to be Session Initiator Thread for this initial response(It's not talking to the device at this point anyways)
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
                bytesAddressPair = UDPServerSocket.recvfrom(BUFFERSIZE)
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
                    if (GET_SAMPLETIMES):
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Hdiff = unpacked[KEY.REALSTART]- mySession[MDK.LHS]
                        
                    InsertHallData(unpacked, mySession)
                    if DISP_PKT_RX:
                            message = "H Data Rx'd (%d Bytes)"%Packet.__sizeof__()
                            message+= "    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed) if GET_SAMPLETIMES else ''
                            message+= "    (Start Delta=%d)"%Hdiff if mySession[MDK.LHS]>=0 else ""
                            print(message, flush = True)
                    mySession[MDK.LHS] = unpacked[KEY.REALSTART]
        
                elif ((mType==msgType.ACCEL)):
                    if (GET_SAMPLETIMES):
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Adiff = unpacked[KEY.REALSTART]-mySession[MDK.LAS]
                
                    InsertAccelData(unpacked, mySession)
                    if DISP_PKT_RX:
                        message = "A Data Rx'd (%d Bytes)"%Packet.__sizeof__()
                        message+= "    ST=%d    ET=%d    Elapsed=%d" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed) if GET_SAMPLETIMES else ''
                        message+= "    (Start Delta=%d)"%Adiff if mySession[MDK.LAS]>=0 else ""
                        print(message, flush = True)
                    
                    mySession[MDK.LAS] = unpacked[KEY.REALSTART]
                    
                elif ((mType==msgType.CAL_HALL)):
                    InsertHallCal(unpacked, mySession)
                    if DISP_PKT_RX:
                        print("Cal Data Rx'd (%d Bytes)"%Packet.__sizeof__())
                elif ((mType==msgType.END)):
                    mySession[MDK.TIME_SE] = unpacked[KEY.ENDSESSION]
                    
                    byeResponse = {KEY.SRVOK: mySession[MDK.LHS]} #Send time stamp of first entry of last packet (probably wont be used)
                    byeBytes =enc_msgpack(byeResponse)
                    UDPServerSocket.sendto(byeBytes, (mySession[MDK.IP],mySession[MDK.PT]))
                    
                    self.shutdown_flag.set()
                else:
                    if DISP_PKT_RX:
                        print("Bad Packet Rx'd from: %s" %address[0])
                        print(Packet)
                        print(unpacked)
                    if mySession !=None:
                        mySession[MDK.BPC]+=1
        if self.orphaned.is_set():
            print("Session #%d closed! (Orphaned)" %(mySession[MDK.ID]),flush = True)
            endSession(mySession,True)
        else:
            print("Session #%d closed! (Completed)" %(mySession[MDK.ID]), flush=True)
            endSession(mySession,False)
        
        finishedSessions.append(mySession)
        START_THREAD.aSessionIsComplete.set()#Notify session initialization thread that we are done

class ServiceBroadcaster(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.shutdown_flag = threading.Event()
    
    def run(self):
        ServiceDictionary = {"SVC":SERVICETXT,"VER":VERSIONNUM,"PORT":START_PORT}
        ServiceMessage = enc_msgpack(ServiceDictionary,use_single_float=True,strict_types=True)
        interfaces = {"127.0.0.2", LOCALIP}#localhost, .2 since windows prohibits .1
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
        BCAST_THREAD.join()
    if START_THREAD != None:
        START_THREAD.shutdown_flag.set()
        START_THREAD.join()
    if KB_THREAD != None:
        KB_THREAD.shutdown_flag.set()
        KB_THREAD.join()
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
    aPort = random.randint(PORTMIN, PORTMAX)
    while(isPortUsed(aPort)):
        aPort = random.randint(PORTMIN, PORTMAX)
        #print("Next RND Port")
    #print(usedPorts)
    return aPort

def isPortUsed(checkport):
    #inList = False
    #for port in usedPorts:
    #    if port == checkport:
    #            inList = True
    #            break
    inList = (checkport in usedPorts)
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

def SaveSettings(quiet):
    #Vars saved as Dict for easy JSON export
    with open(CFG_FNAME, 'w') as outfile:
        json.dump(SVR_CFG, outfile)
    if not quiet:
        print("(Settings Saved)")

def LoadSettings(quiet):
    global SVR_CFG
    if os.path.exists(CFG_FNAME):
        with open(CFG_FNAME) as json_file:
            SVR_CFG = json.load(json_file)
        print("Last Session # loaded from file: %d" %SVR_CFG[CFG.K_SID])   
    else:
        print("No Settings file found, using defaults")
    if not quiet:
        print("(Settings Loaded)")

def newSQLdb(mySession): #Setup the session specific DB
    global sqlConn, cursor
    fName = DB_FNAME + str(mySession[MDK.ID]) + '.db'
    sqlConn = sqlite3.connect(fName, timeout = 10)
    cursor = sqlConn.cursor()
    mySession[MDK.SQP] = sqlConn
    mySession[MDK.SQC] = cursor
    UseHZ = mySession[MDK.HZ]
    NumHallSensors = mySession[MDK.NHS]
    PKT_TIMES = mySession[MDK.TIME_KPT]
    
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
    if PKT_TIMES:
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
    global lastSESSIONID
    cursor = mySession[MDK.SQC]
    sqlConn = mySession[MDK.SQP]
    lastSESSIONID = mySession[MDK.ID]
    mySession[MDK.SC] = 1
    mySession[MDK.TIME_KPT] = KEEP_PKT_TIMES
    if offlineClose:
        print("Orphaned: Using last Data RX'd as Session End Time")
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
    PKT_TIMES = mySession[MDK.TIME_KPT]
    if PKT_TIMES:
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
    PKT_TIMES = mySession[MDK.TIME_KPT]
    if PKT_TIMES:
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

"""def shutdownServer():
    global sqlConn, cursor, Sessions
    SaveSettings() #Store last SESSIONID
    for session in Sessions:
        if (session[MDK.SQP]!=None):#If its a string then no sql db is open
            print("Session #%d not closed correctly! (Closing SQL DB now...)"%session[MDK.ID])
            endSession(session,True)#Perform offline close
            #session[MDK.SQP].close();
            print("\tTotal Bad Packets recieved: %d" %session[MDK.BPC])
    print("Last Active Session: %d"%SESSIONID)
    print("Good Bye")
    sysExit()"""

def clearSettingsAndFiles(quiet):
    global SVR_CFG
    if os.path.exists(CFG_FNAME):
        if not quiet:
            print("RESET\tDeleting old CFG file (%s)" %CFG_FNAME)
        os.remove(CFG_FNAME)
    
    #Since we start with a copy of the default settings, nothing to set here, just save these defaults
    #SVR_CFG = CFG._DEFAULTS
    
    SaveSettings(True)
    
    rawdirectory = os.getcwd() + SVR_CFG[CFG.K_DBRD]
    rawDirOk = checkOrMakeDir(rawdirectory)
    if rawDirOk:
        print("Raw DB Directory present, clearing now.")
        clearDir(rawdirectory, ".db")
    
    reportdirectory = os.getcwd() + SVR_CFG[CFG.K_REPD]
    repDirOk = checkOrMakeDir(rawdirectory)
    
    if repDirOk:
        print("Report Directory present, clearing now.")
        clearDir(rawdirectory, ".pdf")
        clearDir(rawdirectory, ".png")
        clearDir(rawdirectory, ".html")

def checkOrMakeDir(directory):
    dirOk = False
    if os.path.isdir(directory):
        dirOk = True
    else:
        os.mkdir(directory)
        dirOk = True
    if dirOk: #check once again to be sure it created
        dbFileDirHandle = os.listdir( directory )
    return dirOk

def clearDir(directory, fType):
    dbFileDirHandle = os.listdir( directory )
    for item in dbFileDirHandle:
        if item.endswith(fType):
            os.remove( os.path.join( directory, item ) )
            
            
"""
    Main Program Starts here 
"""
def main():
    global SVR_CFG
        
    if SVR_CFG[CFG.K_RAR]:
        clearSettingsAndFiles(True)

    LoadSettings(False)
    
    args = sysArgs
    alen = len(args)
    message = ""
    if alen>=2:
        try:
            print(args)
            if args[1]=="--reset":
                    if not SVR_CFG[CFG.K_RAR]:
                        clearSettingsAndFiles(True)
                        exit()
            message = "The following settings were updated: "
            for item in range(1,alen,2):
                if args[item] == "-ip":
                    SVR_CFG[CFG.K_IP] = args[item +1]
                    message+="\n\tIP Address of Server"
                if args[item] == "-sp":
                    SVR_CFG[CFG.K_STP] = int(args[item +1])
                    message+="\n\tSession Initiator Port"
                if args[item] == "-bp":
                    SVR_CFG[CFG.K_BCP] = int(args[item +1])
                    message+="\n\tService Broadcast Port"
                if args[item] == "-bi":
                    SVR_CFG[CFG.K_BCI] = int(args[item +1])
                    message+="\n\tService Broadcast Interval"
                if args[item] == "-pmax":
                    SVR_CFG[CFG.K_SPU] = int(args[item +1])
                    message+="\n\tSession Maximum Port Number"
                if args[item] == "-pmin":
                    SVR_CFG[CFG.K_SPL] = int(args[item +1])
                    message+="\n\tSession Minimum Port Number"
#END OF DAY JULY 13, STILL WORKING ON CONFIG FILE PARAMETERS AND COMMAND LINE ARGUMENTS
        except:
            print("Invalid arguments specified, check input and try again!")
            sysExit()
            
        print(message)
        SaveSettings(True)
    sysExit()
        
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
    
if __name__ == "__main__":
    main()
    