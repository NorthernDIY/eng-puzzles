#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, msgpack, pprint, sqlite3, sys, os
from time import sleep
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
    INIT = 1
    HALL = 2
    ACCEL = 3

def unpack(UDP_packet):
    message = None
    try:
        message = msgpack.loads(UDP_packet)
    except:
        print("Failed to decode MSGPACK")
        #udp send 0 (Failed)
    else:
        #udp send 1 (good)
        print("MSGPACK Decoded")
    finally:
        return message

def getMsgType(message):
    if message.__contains__("Hello"):
        return msgType.INIT
    elif message.__contains__("TH"):
        return msgType.HALL
    elif message.__contains__("TA"):
        return msgType.ACCEL
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
            print("ResetDB:  Deleting old DB file")
            os.remove(db_fName)
            sleep(1)
        else:
            print("ResetDB:  No DB file to delete (Not an error)")
        
        #Regenerate new sql db file with tables/headers
        setupSQLdb(db_fName)
        print("Done, Exiting...")
        sqlConn.close()
        sys.exit()
    else:
        print("Invalid arguments, only option is --resetdb")
        sys.exit()
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")

# Listen for incoming datagrams
while(True):
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    Packet = bytesAddressPair[0]
    address = bytesAddressPair[1]
    #clientMsg = "Message from Client:{}".format(message)
    #pprint.pprint(Packet)
    
    unpacked = unpack(Packet)
    if type(unpacked) != None:
        mType = getMsgType(unpacked)
        #clientIP  = "Client IP Address:{}".format(address) 
		#print(clientMsg)
		#print(clientIP)
        if (mType==msgType.INIT):
            print("New Session")
            sessionID +=1;
            SIDresponse = {'ID':sessionID}
            msgToClient = msgpack.dumps(SIDresponse)
            UDPServerSocket.sendto(msgToClient, address)
            #pprint.pprint(unpacked)
        elif (mType==msgType.HALL):
            print("H Data Rx'd")
            #pprint.pprint(unpacked)
        elif (mType==msgType.ACCEL):
            print("A Data Rx'd")
        #pprint.pprint(unpacked)
        else:
            print("Bad Packet!")
        # Sending a reply to client
        #UDPServerSocket.sendto(bytesToSend, address)

