#Based off https://pythontic.com/modules/socket/udp-client-server-example

from ast import Load
import socket, msgpack, pprint, sqlite3, sys, os
from types import NoneType
from time import sleep
from datetime import datetime
from colorama import Fore,Back,Style
from enum import Enum
import json

#Network Settings (Defaults)
localIP     = "127.0.0.1"# Localhost for now
localPort   = 20002
bufferSize  = 20000

#SQL Things
sqlConn = None
cursor = None

#FileName Things
db_RawDir = '/RawDat/'
db_fName = '.' + db_RawDir + 'MZRaw_S' #Session db prefix
cfg_fName = 'ServerConfig.json' #Filename of settings storage


#Session Things
sessionID = 0
sessionStartTimeDate = -1
FWVer = -1

#store vars as dict for easy JSON Load/Dump
cfgVars = {"IP": localIP, "Port": localPort, "bSize": bufferSize, "SID":sessionID}


HBSz = 0 #Hall Sensor Block Size
HST = 0 #Hall Sensor Sampling Period
HT = 0 #Hall Sensor Time Stamp (last entry)
NumSensors = 24 #Number of Hall sensors

ABSz = 0 #Accelerometer Block Size
AST = 0 #Accelerometer Sampling Interval
AT = 0 #Accelerometer Time Stamp (last entry)


class msgType(Enum):
    JUNK= 0
    START =1
    END = 2
    HALL = 3
    ACCEL = 4

def unpack(UDP_packet):
    message = None
    try:
        message = msgpack.loads(UDP_packet)
    except:
        print("Failed to decode MSGPACK")
        #udp send 0 (Failed), except we don't care right now
    #else:
        #udp send 1 (good), except we don't care right now
        #print("MSGPACK Decoded")
    finally:
        return message

#Determine message type based on presence of specific Keys
def getMsgType(message):
    #pprint.pprint(message)
    if message.__contains__("Hello"):
        return msgType.START
    elif message.__contains__("TH"):
        return msgType.HALL
    elif message.__contains__("TA"):
        return msgType.ACCEL
    elif message.__contains__("Bye"):
        return msgType.END
    else:
        print("Bad Packet Type!")
        return msgType.JUNK


def SaveSettings():
    global cfgVars
    cfgVars["SID"] = sessionID
    with open(cfg_fName, 'w') as outfile:
        json.dump(cfgVars, outfile)
    #file1 = open(cfg_fName, "wb") 
    #pickle.dump(cfgVars, file1)
    #file1.close

def LoadSettings():
    global cfgVars, sessionID, localIP, localPort, bufferSize
    if os.path.exists(cfg_fName):
        #file1 = open(cfg_fName, "rb") 
        #cfgVars = pickle.load(file1)
        with open(cfg_fName) as json_file:
            cfgVars = json.load(json_file)
        sessionID = cfgVars["SID"]
        localIP = cfgVars["IP"]
        localPort = cfgVars["Port"]
        bufferSize = cfgVars["bSize"]
        print("Last Session # loaded from file: %d" %sessionID)
        #file1.close
    else:
        print("No Settings file found, using defaults")

def newSQLdb(fName): #Setup the session specific DB
    global sqlConn, cursor
    sqlConn = sqlite3.connect(fName, timeout = 10)
    cursor = sqlConn.cursor()
    createParamTable = "CREATE TABLE Params(SessionID INTEGER PRIMARY KEY, StartTime TEXT, IP TEXT, StopTime TEXT, ABS INTEGER, HBS INTEGER, AST INTEGER, HST INTEGER, NumSensors Integer, FWVer TEXT);"
    createHallTable = "CREATE TABLE Hall(senseNum INTEGER, time INTEGER, x INTEGER, y INTEGER, PRIMARY KEY(senseNum, time));"
    createAccelTable = "CREATE TABLE Accel(time integer PRIMARY KEY, x INTEGER, y INTEGER, z INTEGER);"
    cursor.execute(createParamTable)
    cursor.execute(createHallTable)
    cursor.execute(createAccelTable)
    sqlConn.commit()

def InsertHallData(messageData):
    SqlString = "INSERT INTO Hall(senseNum, time, x, y) Values( ?, ?, ?, ?);"
    #HallDict = {}
    currTime = messageData["TH"]
    
    i = 0
    for t in range(0,HBSz):#outerloop = time
        xTag = "x" + str(i)
        yTag = "y" + str(i)
        for s in range(0,NumSensors):#innerloop = sensor
            sqlData = (str(s), str(t*HST + currTime), messageData[xTag][s], messageData[yTag][s])
            cursor.execute(SqlString,sqlData)
        i+=1
        sqlConn.commit()

def InsertAccelData(messageData):
    SqlString = "INSERT INTO Accel(time, x, y, z) Values( ?, ?, ?, ?);"
    currTime = messageData["TA"]
    #JUNE 6/2022 DAY ENDPOINT - NEED TO REFORMAT FUNCTION TO STORE ACCEL DATA IN SQLITE3 DB
    i = 0
    for t in range(0,ABSz):#outerloop = time
        sqlData = (str(t*AST + currTime), messageData["x"][s], messageData[yTag][s])
        cursor.execute(SqlString,sqlData)
        i+=1
        sqlConn.commit()


args = sys.argv
if len(args)==2:
    if args[1]=="--reset":
        if os.path.exists(cfg_fName):
            print("RESET\tDeleting old CFG file (%s)" %cfg_fName)
            os.remove(cfg_fName)
            SaveSettings()
            directory = os.getcwd() + db_RawDir
            test = os.listdir( directory )

            for item in test:
                if item.endswith(".db"):
                    os.remove( os.path.join( directory, item ) )
            print("RESET\tSession DB files Removed")
            sleep(1)
    else:
        print("Invalid arguments, only option is --reset")
        sys.exit()

LoadSettings()
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening @%s:%d" %(localIP, localPort))

#Current IF IP address
print((([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0])

activeSessions = {"0.0.0.0":-1}

# Listen for incoming datagrams
try:
    while(True):
        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
        Packet = bytesAddressPair[0]
        address = bytesAddressPair[1]
        unpacked = unpack(Packet)
        if type(unpacked) != None:
            mType = getMsgType(unpacked)
            
            if (mType==msgType.START):#TODO - Associate SQL conns & cursors to each IP/Session
                if not (activeSessions.__contains__(address[0])):
                    print("New Session started at %s by %s"%(datetime.now(), address[0]))
                    sessionID +=1
                    activeSessions[address[0]] = sessionID
                    SIDresponse = {'ID':sessionID}
                    msgToClient = msgpack.dumps(SIDresponse)
                    UDPServerSocket.sendto(msgToClient, address)
                else:#previous Session not closed, close now, mark for processing, start new session
                    pprint.pprint(activeSessions)
                    lastSession = activeSessions[address[0]]
                    print("Previous session (#%d) by %s was not closed, closing now!" %(lastSession, address[0]))
                    print("NOT IMPLEMENTED - Closure of OLD DB TODO: FIX THIS")
                    #endSession(lastSession) # marks session complete and permits data processing
                    sessionID +=1
                    activeSessions[address[0]] = sessionID
                    print("New session at %s by %s"%(datetime.now(), address[0]))
                    SIDresponse = {'ID':activeSessions[address[0]]}
                    msgToClient = msgpack.dumps(SIDresponse)
                    UDPServerSocket.sendto(msgToClient, address)
                #Setup Session DB
                filename = db_fName + str(sessionID) + '.db'
                newSQLdb(filename)
                HBSz = unpacked["HBS"]
                ABSz = unpacked["ABS"]
                AST = unpacked["AST"]
                HST = unpacked["HST"]
                FWVer = unpacked["Hello"]
                NumSensors=unpacked["NumSens"]
                sessionStartTimeDate = datetime.now()

            elif (mType==msgType.END):
                print("Session #%d closed!" %(activeSessions[address[0]]))
                lastSessionID = activeSessions[address[0]]
                #"CREATE TABLE Params(SessionID INTEGER PRIMARY KEY, StartTime TEXT, IP TEXT, StopTime TEXT, ABS INTEGER, HBS INTEGER, AST INTEGER, HST INTEGER);"
                sqlCmd = "INSERT INTO Params(SessionID,StartTime, IP, StopTime, ABS, HBS, AST, HST, NumSensors, FWVer ) VALUES ("+ str(sessionID) +", '" + str(sessionStartTimeDate) + "', '" + address[0] + "', '"+ str(datetime.now()) +"', " +str(ABSz) + ", " + str(HBSz) +", " + str(AST) +", " + str(HST) + ", " +str(NumSensors) + ", '" +str(FWVer)  +"');"
                #print(sqlCmd)
                cursor.execute(sqlCmd)
                sqlConn.commit()
                sqlConn.close()
                sqlConn = None
                #markSessionComplete(lastSessionID)
                del activeSessions[address[0]]

            elif (mType==msgType.HALL):
                print("H Data Rx'd")
                InsertHallData(unpacked)

                #pprint.pprint(unpacked)
            elif (mType==msgType.ACCEL):
                print("A Data Rx'd")
                #pprint.pprint(unpacked)
            else:
                print("Bad Packet Rx'd from: %s" %address[0])
            # Sending a reply to client
            #UDPServerSocket.sendto(bytesToSend, address)
except KeyboardInterrupt:
    print("\nKeyboard Interrupt - Shutting Down")
    SaveSettings() #Store last SessionID
    if (type(sqlConn)!=NoneType):#If its a string then no sql db is open
        sqlConn.close()
    sys.exit()
