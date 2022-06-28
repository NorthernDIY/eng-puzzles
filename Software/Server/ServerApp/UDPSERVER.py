#!/bin/python3
#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, msgpack, sqlite3, os
from msgpack import loads as dec_msgpack
from msgpack import dumps as enc_msgpack
from sys import argv as sysArgs
from sys import exit as sysExit
#from types import NoneType
from time import sleep
from datetime import datetime
#from colorama import Fore,Back,Style
from enum import Enum
from pprint import pprint
import json
import subprocess
import platform
#import keyboard
#Things you might want to change
#Network Settings (Defaults)
localIP_d     = "127.0.0.1"# Localhost for now
localPort_d   = 20002
bufferSize_d  = 20000

"""
Todo:	-Fix missing RawDat directory error
		-Detect current OS vs Last OS at startup and autoreset if different (Helpful for development)
		-Watch Session ID of incoming packet and discard if not associated with IP (Basic anti-spoofing)
		-Figure out how to close out sessions after some time of no activity --or- when next session started by same IP
		-Netifaces get all interfaces, no joke stop skipping things randomly
		
"""

#FileName Things
db_RawDir = '/RawDat/'
db_fName = '.' + db_RawDir + 'MZRaw_S' #Session db prefix
cfg_fName = 'ServerConfig.json' #Filename of settings storage

#Client Device parameter Adjustments
#At session start, server sends the following parameters to the ESP32
ServerWantsHZ = 0	#(Sent to ESP32) 1 = Send Hall Z data, 0 = Don't send
HST = 10 			#(Sent to ESP32)Hall Sensor Sampling Period
HRT = 100			#(Sent to ESP32)Hall Sensor Report time - [Interval in which MA-Filtered Hall data is sent]
HAVS = 20	#MAX=60	#(Sent to ESP32)Hall Sensor # of entries in Moving Average Filter
AST = 40 			#(Sent to ESP32)Accelerometer Sampling Interval

#Run Time Variables
#Network setting variables
localIP     = localIP_d
localPort   = localPort_d
bufferSize  = bufferSize_d

#SQL Things
sqlConn = None
cursor = None
#Session Things
sessionID = 0
sessionStartTimeDate = -1
FWVer = -1
badPacketCount = 0

HBSz = 0 	#(Rx'd from ESP32) Hall Sensor Block Size
 

HT = 0 #(Rx'd from ESP32) Hall Sensor Time Stamp (last entry)
NumSensors = 0 #(Rx'd from ESP32)Number of Hall sensors

currHTime = 0 #(Rx'd from ESP32) Timestamp of last recieved Hall data packet (first entry)

ABSz = 0 #(Rx'd from ESP32 )Accelerometer Block Size

AT = 0 #(Rx'd from ESP32) Accelerometer Time Stamp (last entry)

SIP = False
UseHZ = False #Capture Hall Z values (Off by Default)


HUSecLast = 0
AUSecLast = 0

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
        #udp send 0 (Failed), except we don't care right now
    #else:
        #udp send 1 (good), except we don't care right now
        #print("MSGPACK Decoded")
    finally:
        return message

#Determine message type based on presence of specific Keys
def getMsgType(message):
    if (message != None):
        if message.__contains__("Hello"):
            return msgType.START
        elif message.__contains__("TH"):
            return msgType.HALL
        elif message.__contains__("TA"):
            return msgType.ACCEL
        elif message.__contains__("Bye"):
            return msgType.END
        elif message.__contains__("BSX"):
            return msgType.CAL_HALL
        elif message.__contains__("CHK"):
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


def newSQLdb(fName): #Setup the session specific DB
    global sqlConn, cursor
    sqlConn = sqlite3.connect(fName, timeout = 10)
    cursor = sqlConn.cursor()
    createParamTable = "CREATE TABLE Params(SessionID INTEGER PRIMARY KEY, StartTime TEXT, IP TEXT, StopTime TEXT, ABS INTEGER, HBS INTEGER, AST INTEGER, HST INTEGER, NumSensors Integer, FWVer TEXT, USEHZ Integer);"
    if UseHZ:
        createHallTable = "CREATE TABLE Hall(time INTEGER, senseNum INTEGER, x DECIMAL, y DECIMAL, z DECIMAL, PRIMARY KEY(senseNum, time));"
        createHallCalNoiseTable = "CREATE TABLE HallCalNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, z DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HallCalBS(BSX DECIMAL, BSY DECIMAL, BSZ DECIMAL);"
    else:
        createHallTable = "CREATE TABLE Hall(time INTEGER, senseNum INTEGER, x DECIMAL, y DECIMAL, PRIMARY KEY(senseNum, time));"
        createHallCalNoiseTable = "CREATE TABLE HallCalNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HallCalBS(BSX DECIMAL, BSY DECIMAL);"
    createAccelTable = "CREATE TABLE Accel(time integer PRIMARY KEY, x DECIMAL, y DECIMAL, z DECIMAL);"
    cursor.execute(createParamTable)
    cursor.execute(createHallTable)
    cursor.execute(createHallCalNoiseTable)
    cursor.execute(createHallCalBSTable)
    cursor.execute(createAccelTable)
    sqlConn.commit()


def InsertHallData(messageData):
    global currHTime
    if UseHZ:
        SqlString = "INSERT INTO Hall(time, senseNum, x, y, z) Values( ?, ?, ?, ?, ?);"
    else:
        SqlString = "INSERT INTO Hall(time, senseNum, x, y) Values( ?, ?, ?, ?);"
    #HallDict = {}
    currHTime = messageData["TH"] 
    i = 0
    for t in range(0,HBSz):#outerloop = time
        xTag = "x" + str(i)
        yTag = "y" + str(i)
        if UseHZ:
            zTag = "z" + str(i)
        for s in range(0,NumSensors):#innerloop = sensor
            if UseHZ:
                sqlData = (str(t*HST + currHTime),str(s), messageData[xTag][s], messageData[yTag][s], messageData[zTag][s])
            else:             
                sqlData = (str(t*HST + currHTime),str(s), messageData[xTag][s], messageData[yTag][s])
            cursor.execute(SqlString,sqlData)
        i+=1
        sqlConn.commit()

def InsertHallCal(messageData):
    if UseHZ:
        SqlString_a = "INSERT INTO HallCalNoise(senseNum, x, y, z) Values( ?, ?, ?, ?);"
        SqlString_b = "INSERT INTO HallCalBS(BSX, BSY, BSZ) Values( ?, ?, ?);"
    else:
        SqlString_a = "INSERT INTO HallCalNoise(senseNum, x, y) Values( ?, ?, ?);"
        SqlString_b = "INSERT INTO HallCalBS(BSX, BSY) Values( ?, ?);"
    for s in range(0,NumSensors):#innerloop = sensor
        if UseHZ:
            sqlData_a = (str(s), messageData["H_CAL_X"][s], messageData["H_CAL_Y"][s], messageData["H_CAL_Z"][s])
            
        else:             
            sqlData_a = (str(s), messageData["H_CAL_X"][s], messageData["H_CAL_Y"][s])
        cursor.execute(SqlString_a,sqlData_a)
        sqlConn.commit()
    
    if UseHZ:
        sqlData_b = (messageData["BSX"], messageData["BSY"], messageData["BSZ"])
    else:
        sqlData_b = (messageData["BSX"], messageData["BSY"])
    cursor.execute(SqlString_b,sqlData_b)
    sqlConn.commit()


def InsertAccelData(messageData):
    SqlString = "INSERT INTO Accel(time, x, y, z) Values( ?, ?, ?, ?);"
    currTime = messageData["TA"]
    i = 0
    for t in range(0,ABSz):#outerloop = time
        sqlData = (str(t*AST + currTime), messageData["x"][t], messageData["y"][t], messageData["z"][t])
        #pprint.pprint(sqlData)
        cursor.execute(SqlString,sqlData)
        i+=1
        sqlConn.commit()

def shutdownServer():
    SaveSettings() #Store last SessionID
    if (sqlConn!=None):#If its a string then no sql db is open
        sqlConn.close()
    #ServiceBroadcaster.terminate()
    print("Total Bad Packets recieved: %d" %badPacketCount)
    sysExit()




####################################
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
        if os.path.exists(cfg_fName):
            print("RESET\tDeleting old CFG file (%s)" %cfg_fName)
            os.remove(cfg_fName)
            sessionID = 0
            localIP     = "127.0.0.1"# Localhost for now
            localPort   = 20002
            bufferSize  = 20000
            SaveSettings()
            directory = os.getcwd() + db_RawDir
            test = os.listdir( directory )
            for item in test:
                if item.endswith(".db"):
                    os.remove( os.path.join( directory, item ) )
            print("RESET\tSession DB files Removed")
            sleep(1)
elif alen==1:
            print("Using settings loaded from file")
else:
    print("Invalid arguments: Specify --ip x.x.x.x & --port nnnnn or --reset")
    sysExit()



#ServiceBroadcaster = threading.Thread(target=broadCastService,args=(7777,)).start()
#ServiceBroadcaster = subprocess.Popen(["python","brdcast.py"],shell=False)

#print("After broadcaster")
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
#UDPServerBroadcastSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
#UDPServerBroadcastSocket.bind(('', 8675))
print("UDP server up and listening @%s:%d" %(localIP, localPort))

#Current IF IP address
#print((([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0])

activeSessions = {"0.0.0.0":-1} # Guard value, not really needed

    
try:# Try allows us to catch keyboard interrupt
    while(True):
        
        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    
        Packet = bytesAddressPair[0]
        
        address = bytesAddressPair[1]
        unpacked = unpack(Packet)
        #pprint(unpacked)
        if unpacked != None:
            mType = getMsgType(unpacked)
            
            if (mType==msgType.START):#TODO - Associate SQL conns & cursors to each IP/Session
                if not (activeSessions.__contains__(address[0])):
                    sessionID +=1
                    print("New Session #%d started at %s by %s"%(sessionID,datetime.now(), address[0]),flush=True)
                    activeSessions[address[0]] = sessionID
                    SIDresponse = {'ID':sessionID,'WantHZ':ServerWantsHZ,'AST':AST,'HST':HST,'HRT':HRT,'HMS':HAVS}
                    msgToClient = enc_msgpack(SIDresponse)
                    UDPServerSocket.sendto(msgToClient, address)
                else:#previous Session not closed, close now, mark for processing, start new session
                    pprint(activeSessions)
                    lastSession = activeSessions[address[0]]
                    print("Previous session (#%d) by %s was not closed, closing now!" %(lastSession, address[0]))
                    print("NOT IMPLEMENTED - Closure of OLD DB TODO: FIX THIS")
                    #endSession(lastSession) # marks session complete and permits data processing
                    sessionID +=1
                    activeSessions[address[0]] = sessionID
                    print("New session #%d at %s by %s"%(sessionID, datetime.now(), address[0]),flush=True)
                    SIDresponse = {'ID':activeSessions[address[0]],'WantHZ':ServerWantsHZ,'AST':AST,'HST':HST,'HRT':HRT,'HMS':HAVS}
                    msgToClient = enc_msgpack(SIDresponse)
                    UDPServerSocket.sendto(msgToClient, address)
                #Setup Session DB
                pprint(SIDresponse)
                filename = db_fName + str(sessionID) + '.db'
                HBSz = unpacked["HBS"]
                ABSz = unpacked["ABS"]
                #AST = unpacked["AST"]
                #HST = unpacked["HST"]
                FWVer = unpacked["Hello"]
                NumSensors=unpacked["NumSens"]
                UseHZ = (unpacked["HallHasZ"]==1)&(ServerWantsHZ==1)
				
                newSQLdb(filename)
                sessionStartTimeDate = datetime.now()
                #pprint.pprint(unpacked)

            elif (mType==msgType.END):
                print("Session #%d closed!" %(activeSessions[address[0]]), flush=True)
                lastSessionID = activeSessions[address[0]]
                sqlCmd = "INSERT INTO Params(SessionID,StartTime, IP, StopTime, ABS, HBS, AST, HST, NumSensors, FWVer, UseHZ ) VALUES ("+ str(sessionID) +", '" + str(sessionStartTimeDate) + "', '" + address[0] + "', '"+ str(datetime.now()) +"', " +str(ABSz) + ", " + str(HBSz) +", " + str(AST) +", " + str(HST) + ", " +str(NumSensors) + ", '" +str(FWVer)  +"'," +str(UseHZ) +");"
                cursor.execute(sqlCmd)
                sqlConn.commit()
                sqlConn.close()
                sqlConn = None
                #markSessionComplete(lastSessionID)  #Probably just going to go off of file open or not!
                del activeSessions[address[0]]
                byeResponse = {"OK": currHTime} #Send time stamp of first entry of last packet (probably wont be used)
                byeBytes =enc_msgpack(byeResponse)
                UDPServerSocket.sendto(byeBytes, address)
            elif (mType==msgType.CHECK):
                chkResponse = {"OK": 1}
                chkBytes =enc_msgpack(chkResponse)
                UDPServerSocket.sendto(chkBytes, address)
                print("ESP checked server alive!")
            elif (mType==msgType.HALL):
                print("H Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end = "")
                elapsed = unpacked["E"] - unpacked["S"]
                Hdiff = unpacked["S"]-HUSecLast
                print("    ST=%d    ET=%d    Elapsed=%d    (Start Delta=%d)" %(unpacked["S"], unpacked["E"], elapsed,Hdiff))
                HUSecLast = unpacked["S"]
                #pprint(unpacked)
                InsertHallData(unpacked)

            elif (mType==msgType.ACCEL):
                print("A Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end ="")
                elapsed = unpacked["E"] - unpacked["S"]
                Adiff = unpacked["S"]-AUSecLast
                print("    ST=%d    ET=%d    Elapsed=%d    (Start Delta=%d)" %(unpacked["S"], unpacked["E"],elapsed, Adiff))
                AUSecLast = unpacked["S"]
				#pprint(unpacked)
                #print("\n\n",flush=True)
                InsertAccelData(unpacked)
            elif (mType==msgType.CAL_HALL):
                InsertHallCal(unpacked)
                print("Cal Data Rx'd (%d Bytes)"%Packet.__sizeof__())
            else:
                print("Bad Packet Rx'd from: %s" %address[0])
                badPacketCount+=1
                #maybe send some data...or not
                #perhaps implement some sort of flag to check if the next packet is the time step after
                #then just auto fill gap with interpolated values...or fix it in post(processing)
                #UDPServerSocket.sendto(bytesToSend, address)
    print("Shutting down")
    shutdownServer()
    
except KeyboardInterrupt:
    print("\nKeyboard Interrupt - Shutting Down", flush=True)
    shutdownServer()
    