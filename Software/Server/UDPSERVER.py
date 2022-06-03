#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, msgpack, pprint, sqlite3, sys, os
from time import sleep
from datetime import datetime
from colorama import Fore,Back,Style
from enum import Enum

#Variables
localIP     = "127.0.0.1"# Localhost for now
localPort   = 20002
bufferSize  = 20000

#SQL Things
sqlConn = ''
cursor = ''
db_fName = 'SeriousGamesRaw.db'

#Session Things
sessionID = 0

HBSz = 0 #Hall Sensor Block Size
HT = 0 #Hall Sensor Time Stamp (last entry)
HSensors = 24 #Number of Hall sensors

ABSz = 0 #Accelerometer Block Size
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
        #udp send 0 (Failed)
    #else:
        #udp send 1 (good)
        #print("MSGPACK Decoded")
    finally:
        return message

#Determine message type based on presence of specific Keys
def getMsgType(message):
    pprint.pprint(message)
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

def setupSQLdb(fName):
    global sqlConn, cursor
    sqlConn = sqlite3.connect(fName, timeout = 10)
    cursor = sqlConn.cursor()
    createParamTable = "CREATE TABLE Params(sessionID INTEGER PRIMARY KEY, datetime TEXT);"
    createHallTable = "CREATE TABLE Hall(senseNum INTEGER, time INTEGER, x INTEGER, y INTEGER, PRIMARY KEY(senseNum, time));"
    createAccelTable = "CREATE TABLE Accel(time integer PRIMARY KEY, x INTEGER, y INTEGER, z INTEGER);"
    cursor.execute(createHallTable)
    cursor.execute(createAccelTable)
    sqlConn.commit()
    
 
#msgFromServer       = "Hello UDP Client"
#bytesToSend         = str.encode(msgFromServer)

args = sys.argv
if len(args)==2:
    if args[1]=="--resetdb":
        if os.path.exists(db_fName):
            print("Deleting old DB file (%s)" %db_fName)
            os.remove(db_fName)
            sleep(1)
    else:
        print("Invalid arguments, only option is --resetdb")
        sys.exit()

if os.path.exists(db_fName):
    sqlConn = sqlite3.connect(db_fName, timeout = 10)
    cursor = sqlConn.cursor()
    print("Database opened (%s)" %db_fName)
else:
    print("No DB file found, recreating...", end = "")
    setupSQLdb(db_fName)
    print("Done")

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
        #clientMsg = "Message from Client:{}".format(message)
        #pprint.pprint(Packet)
        
        unpacked = unpack(Packet)
        if type(unpacked) != None:
            mType = getMsgType(unpacked)
            
            if (mType==msgType.START):
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
                    #endSession(lastSession) # marks session complete and permits data processing
                    sessionID +=1
                    activeSessions[address[0]] = sessionID
                    print("New session at %s by %s"%(datetime.now(), address[0]))
                    SIDresponse = {'ID':activeSessions[address[0]]}
                    msgToClient = msgpack.dumps(SIDresponse)
                    UDPServerSocket.sendto(msgToClient, address)

            elif (mType==msgType.END):
                print("Session #%d closed!" %(activeSessions[address[0]]))
                lastSessionID = activeSessions[address[0]]
                #markSessionComplete(lastSessionID)
                del activeSessions[address[0]]

            elif (mType==msgType.HALL):
                print("H Data Rx'd")
                pprint.pprint(unpacked)
            elif (mType==msgType.ACCEL):
                print("A Data Rx'd")
            #pprint.pprint(unpacked)
            else:
                print("Bad Packet Rx'd from: %s" %address[0])
            # Sending a reply to client
            #UDPServerSocket.sendto(bytesToSend, address)
except KeyboardInterrupt:
    print("\nShutting Down")
    sqlConn.close()
    sys.exit()
