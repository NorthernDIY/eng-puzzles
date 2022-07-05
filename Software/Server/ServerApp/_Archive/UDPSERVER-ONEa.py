#!/bin/python3
#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, sqlite3,os
import AMazeThing_Keys as KEY
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






#SQL Things
sqlConn = None
cursor = None
sqlCursor = None

#Session MetaData 
Session = {KEY.SESSIONID:-1, "IP":"0.0.0.0","Started":-1,"Stopped":-1, KEY.ACCELBLOCKSIZE:-1,
           KEY.ACCELSAMPLETIME:-1,KEY.HALLBLOCKSIZE:-1, KEY.HALLSAMPLETIME:-1, KEY.HALLMASIZE:-1,
           KEY.HALLREPORTTIME:-1,"USEHZ":-1, KEY.NUMHALLSENSORS:-1, KEY.FWVER:"N/A", "BadPacketCount":0,
           KEY.STARTSESSION:-1,KEY.ENDSESSION:-1, "SQLConn":sqlConn, "SQLCursor":sqlCursor, "Closed":False}

Sessions = [Session]
#Client Device parameter Adjustments
#At session start, server sends the following parameters to the ESP32
ServerWantsHZ = 1	#(Sent to ESP32) 1 = Send Hall Z data, 0 = Don't send
ServerWantsSTS = 0  #(Sent to ESP32) 1 = Send sample time start/end time stamps, 0 = don't send
HST = 10 			#(Sent to ESP32)Hall Sensor Sampling Period
HRT = 100			#(Sent to ESP32)Hall Sensor Report time - [Interval in which MA-Filtered Hall data is sent]
HAVS = 20	#MAX=60	#(Sent to ESP32)Hall Sensor # of entries in Moving Average Filter
AST = 40 			#(Sent to ESP32)Accelerometer Sampling Interval

FWVer = -1      #(Rx'd from ESP32)
HBSz = 0        #(Rx'd from ESP32) Hall Sensor Block Size
HT = 0          #(Rx'd from ESP32) Hall Sensor Time Stamp (last entry)
NumSensors = 0  #(Rx'd from ESP32)Number of Hall sensors
currHTime = 0   #(Rx'd from ESP32) Timestamp of last recieved Hall data packet (first entry)
ABSz = 0        #(Rx'd from ESP32 )Accelerometer Block Size
AT = 0          #(Rx'd from ESP32) Accelerometer Time Stamp (last entry)
UseHZ = False   #(Set above by ServerWantsHZ)Capture Hall Z values (Off by Default)


#Session Things
sessionID = 0
sessionStartTimeDate = -1

#Interesting things for development purposes
esp32StartMicros = 0
esp32StopMicros = 0

#IF ServerWantsSTS, these store last sample packet start times
HUSecLast = 0
AUSecLast = 0

badPacketCount = 0  #If msgPack Decode fails, this is incremented
lastPacketSystemTime = 0

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


def newSQLdb(fName): #Setup the session specific DB
    global sqlConn, cursor
    sqlConn = sqlite3.connect(fName, timeout = 10)
    cursor = sqlConn.cursor()
    createParamTable = "CREATE TABLE Params(SessionID INTEGER PRIMARY KEY, IP TEXT, StartTime TEXT, StopTime TEXT, ABS INTEGER, AST INTEGER, HBS INTEGER, HST INTEGER, HallMASize Integer, HRT Integer, USEHZ Integer, NumSensors Integer, FWVer TEXT, BadPacketCount Integer, ESP_START Integer, ESP_STOP Integer);"

    createHallTable = "CREATE Table Hall(time INTEGER"
    createHallTable += ",x0 DECIMAL,x1 DECIMAL,x2 DECIMAL,x3 DECIMAL,x4 DECIMAL,x5 DECIMAL,x6 DECIMAL,x7 DECIMAL,x8 DECIMAL,x9 DECIMAL,x10 D1ECIMAL,x11 DECIMAL,"
    createHallTable += "x12 DECIMAL,x13 DECIMAL,x14 DECIMAL,x15 DECIMAL,x16 DECIMAL,x17 DECIMAL,x18 DECIMAL,x19 DECIMAL,x20 DECIMAL,x21 DECIMAL,x22 DECIMAL,x23 DECIMAL"
    createHallTable += ",y0 DECIMAL,y1 DECIMAL,y2 DECIMAL,y3 DECIMAL,y4 DECIMAL,y5 DECIMAL,y6 DECIMAL,y7 DECIMAL,y8 DECIMAL,y9 DECIMAL,y10 D1ECIMAL,y11 DECIMAL,"
    createHallTable += "y12 DECIMAL,y13 DECIMAL,y14 DECIMAL,y15 DECIMAL,y16 DECIMAL,y17 DECIMAL,y18 DECIMAL,y19 DECIMAL,y20 DECIMAL,y21 DECIMAL,y22 DECIMAL,y23 DECIMAL"
    if UseHZ:
        createHallTable += ",z0 DECIMAL,z1 DECIMAL,z2 DECIMAL,z3 DECIMAL,z4 DECIMAL,z5 DECIMAL,z6 DECIMAL,z7 DECIMAL,z8 DECIMAL,z9 DECIMAL,z10 D1ECIMAL,z11 DECIMAL,"
        createHallTable += "z12 DECIMAL,z13 DECIMAL,z14 DECIMAL,z15 DECIMAL,z16 DECIMAL,z17 DECIMAL,z18 DECIMAL,z19 DECIMAL,z20 DECIMAL,z21 DECIMAL,z22 DECIMAL,z23 DECIMAL"
        
        createHallCalNoiseTable = "CREATE TABLE HallCalNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, z DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HallCalBS(BSX DECIMAL, BSY DECIMAL, BSZ DECIMAL);"
    else:
        createHallCalNoiseTable = "CREATE TABLE HallCalNoise(senseNum INTEGER, x DECIMAL, y DECIMAL, PRIMARY KEY(senseNum));"
        createHallCalBSTable = "CREATE TABLE HallCalBS(BSX DECIMAL, BSY DECIMAL);"
    
    createHallTable += ",Primary KEY(time));"
    
    createAccelTable = "CREATE TABLE Accel(time integer PRIMARY KEY, x DECIMAL, y DECIMAL, z DECIMAL);"
    #print (createHallTable)
    #print (createParamTable)
    #print (createHallCalNoiseTable)
    #print (createHallCalBSTable)
    #print (createAccelTable)
    
    cursor.execute(createParamTable)
    cursor.execute(createHallTable)
    
    cursor.execute(createHallCalNoiseTable)
    cursor.execute(createHallCalBSTable)
    cursor.execute(createAccelTable)
    sqlConn.commit()


def InsertHallData(messageData):
    global currHTime
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

def InsertHallCal(messageData):
    
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


def InsertAccelData(messageData):

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
    global sqlConn, cursor
    SaveSettings() #Store last SessionID
    if (sqlConn!=None):#If its a string then no sql db is open
        sqlConn.close()
    print("Total Bad Packets recieved: %d" %badPacketCount)
    sysExit()


"""
    Main Program Starts here
    
"""
def main():
    global localIP, localPort, bufferSize, packet, addresses
    global FWVer, HBSz, HT, NumSensors, currHTime, ABSz, AT, UseHZ, sessionID, sessionStartTimeDate
    global esp32StartMicros, esp32StopMicros, sqlConn, cursor, HUSecLast, AUSecLast, badPacketCount, lastPacketSystemTime
    #global AUSecLast,HUSecLast,activeSessions,sqlConn,cursor, lastSessionID, badPacketCount, HBSz, ABSz
    #global localIP, localPort, bufferSize, lastPacketSystemTime, sessionID, NumSensors, esp32StartMicros, esp32StopMicros
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
                dbDirOk = False
                if os.path.isdir(directory):
                    print("Session DataBase Directory present")
                    dbDirOk = True
                else:
                    print("Session DataBase Directory Missing. Recreating Now!")
                    os.mkdir(directory)
                    dbDirOk = True
                if dbDirOk:
                    dbFileDirHandle = os.listdir( directory )
                    for item in dbFileDirHandle:
                        if item.endswith(".db"):
                            os.remove( os.path.join( directory, item ) )
                            print("RESET\tSession DB files Removed")
                else:
                    print("Could not create session DataBase Directory. Check permissions and/or drive space!")
                exit()
    elif alen==1:
                print("Using settings loaded from file")
    else:
        print("Invalid arguments: Specify --ip x.x.x.x & --port nnnnn or --reset")
        sysExit()
    
    
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((localIP, localPort))
    
    print("UDP server up and listening @%s:%d" %(localIP, localPort))
    
    activeSessions = {"0.0.0.0":-1} # Guard value, not really needed
        
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
                if (mType ==msgType.ACCEL or mType ==msgType.HALL or mType==msgType.CAL_HALL or mType==msgType.END):
                    if (unpacked[KEY.SESSIONID]==sessionID):
                        lastPacketSystemTime = datetime.now()
                        sessionMatch = True
                    else:
                        print("Session ID mismatch, discarding!")
                if (mType==msgType.START):#TODO - Associate SQL conns & cursors to each IP/Session
                    if not (activeSessions.__contains__(address[0])):
                        sessionID +=1
                        print("New Session #%d started at %s by %s"%(sessionID,datetime.now(), address[0]),flush=True)
                        activeSessions[address[0]] = sessionID
                        SIDresponse = {KEY.SESSIONID:sessionID,KEY.SENDHZ:ServerWantsHZ,KEY.ACCELSAMPLETIME:AST,KEY.HALLSAMPLETIME:HST,KEY.HALLREPORTTIME:HRT,KEY.HALLMASIZE:HAVS,KEY.SENDSTS:ServerWantsSTS}
                        msgToClient = enc_msgpack(SIDresponse)
                        UDPServerSocket.sendto(msgToClient, address)
                    else:#previous Session not closed, close now, mark for processing, start new session
                        #pprint(activeSessions)
                        lastSession = activeSessions[address[0]]
                        
                        print("Previous session (#%d) by %s was not closed, closing now!" %(lastSession, address[0]))
                        print("NOT IMPLEMENTED - Closure of OLD DB TODO: FIX THIS")
                        #endSession(lastSession) # marks session complete and permits data processing
                        sessionID +=1
                        activeSessions[address[0]] = sessionID
                        print("New session #%d at %s by %s"%(sessionID, datetime.now(), address[0]),flush=True)
                        #SIDresponse = {'ID':activeSessions[address[0]],'WantHZ':ServerWantsHZ,'AST':AST,'HST':HST,'HRT':HRT,'HMS':HAVS,'STS':ServerWantsSTS}
                        SIDresponse = {KEY.SESSIONID:sessionID,KEY.SENDHZ:ServerWantsHZ,KEY.ACCELSAMPLETIME:AST,KEY.HALLSAMPLETIME:HST,KEY.HALLREPORTTIME:HRT,KEY.HALLMASIZE:HAVS,KEY.SENDSTS:ServerWantsSTS}
                        msgToClient = enc_msgpack(SIDresponse)
                        UDPServerSocket.sendto(msgToClient, address)
                    #Setup Session DB
                    pprint(SIDresponse)
                    filename = db_fName + str(sessionID) + '.db'
                    HBSz = unpacked[KEY.HALLBLOCKSIZE]
                    ABSz = unpacked[KEY.ACCELBLOCKSIZE]
                    FWVer = unpacked[KEY.FWVER]
                    esp32StartMicros = unpacked[KEY.STARTSESSION]
                    NumSensors=unpacked[KEY.NUMHALLSENSORS]
                    UseHZ = (unpacked[KEY.HALLHASZ]==1)&(ServerWantsHZ==1)
                    sessionStartTimeDate = datetime.now()
                    newSQLdb(filename)
                    
    
                elif ((mType==msgType.END) and sessionMatch):
                    print("Session #%d closed!" %(activeSessions[address[0]]), flush=True)
                    lastSessionID = activeSessions[address[0]]
                    esp32StopMicros = unpacked[KEY.ENDSESSION]
                    sqlCmd = "INSERT INTO Params(SessionID, IP, StartTime, StopTime, ABS, AST, HBS, HST, HallMASize, HRT, UseHZ, NumSensors, FWVer, BadPacketCount, ESP_START, ESP_STOP) VALUES ("
                    sqlData = str(sessionID) +",'" + address[0] + "','" + str(sessionStartTimeDate) + "','"+ str(datetime.now()) +"'," +str(ABSz) + "," + str(AST) +"," + str(HBSz) +"," + str(HST) + "," +str(HAVS)  + "," +str(HRT) + "," +str(UseHZ) + "," +str(NumSensors) + ",'" +str(FWVer)  +"'," +str(badPacketCount)+"," +str(esp32StartMicros)+"," +str(esp32StopMicros)
                    sqlCmd += sqlData + ");"
                    cursor.execute(sqlCmd)
                    sqlConn.commit()
                    sqlConn.close()
                    sqlConn = None
                    #markSessionComplete(lastSessionID)  #Probably just going to go off of file open or not!
                    del activeSessions[address[0]]
                    byeResponse = {KEY.SRVOK: currHTime} #Send time stamp of first entry of last packet (probably wont be used)
                    byeBytes =enc_msgpack(byeResponse)
                    UDPServerSocket.sendto(byeBytes, address)
                elif (mType==msgType.CHECK):
                    chkResponse = {KEY.SRVOK: 1}
                    chkBytes =enc_msgpack(chkResponse)
                    UDPServerSocket.sendto(chkBytes, address)
                    print("ESP checked server alive!")
                elif ((mType==msgType.HALL) and sessionMatch):
                    
                    if (ServerWantsSTS):
                        print("H Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end = "")
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Hdiff = unpacked[KEY.REALSTART]-HUSecLast
                        print("    ST=%d    ET=%d    Elapsed=%d    (Start Delta=%d)" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP], elapsed,Hdiff),flush=True)
                        HUSecLast = unpacked[KEY.REALSTART]
                    else:
                        print("H Data Rx'd (%d Bytes)"%Packet.__sizeof__(), flush=True)
                    #pprint(unpacked)
                    InsertHallData(unpacked)
    
                elif ((mType==msgType.ACCEL) and sessionMatch):
                    
                    if (ServerWantsSTS):
                        print("A Data Rx'd (%d Bytes)"%Packet.__sizeof__(), end ="")
                        elapsed = unpacked[KEY.REALSTOP] - unpacked[KEY.REALSTART]
                        Adiff = unpacked[KEY.REALSTART]-AUSecLast
                        print("    ST=%d    ET=%d    Elapsed=%d    (Start Delta=%d)" %(unpacked[KEY.REALSTART], unpacked[KEY.REALSTOP],elapsed, Adiff),flush=True)
                        AUSecLast = unpacked[KEY.REALSTART]
                    else:
                        print("A Data Rx'd (%d Bytes)"%Packet.__sizeof__(), flush=True)
                    #pprint(unpacked)
                    #print("\n\n",flush=True)
                    InsertAccelData(unpacked)
                    
                elif ((mType==msgType.CAL_HALL) and sessionMatch):
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

if __name__ == "__main__":
    main()