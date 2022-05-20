#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, msgpack, pprint, sqlite3
from enum import Enum
class msgType(Enum):
    JUNK= 0
    INIT = 1
    HALL = 2
    ACCEL = 3

def unpack(UDP_packet):
    message = NULL
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
    if message.has_key("Hello"):
        return msgType.INIT
    elif message.has_key("TH"):
        return msgType.HALL
    elif message.has_key("TA"):
        return msgType.ACCEL
    else:
        print("Bad Packet Type!")
        return msgType.JUNK


localIP     = "127.0.0.1"# Localhost for now
localPort   = 20002
bufferSize  = 20000
 
msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)

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
    unpacked = unpack(Packet)
    #clientIP  = "Client IP Address:{}".format(address) 
    #print(clientMsg)
    #print(clientIP)
    mType = getMsgType(unpacked)
    if (mType==msgType.INIT):
        print("New Session")
        pprint.pprint(unpacked)
    elif (mType==msgType.HALL):
        print("H Data Rx'd")
        pprint.pprint(unpacked)
    elif (mType==msgType.ACCEL):
        print("A Data Rx'd")
        pprint.pprint(unpacked)
    else:
        print("Bad Packet!")

    # Sending a reply to client
    UDPServerSocket.sendto(bytesToSend, address)

