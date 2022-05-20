#Based off https://pythontic.com/modules/socket/udp-client-server-example

import socket, msgpack, pprint
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
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    clientMsg = "Message from Client:{}".format(message)
    unpacked = msgpack.loads(message)
    clientIP  = "Client IP Address:{}".format(address) 
    print(clientMsg)
    print(clientIP)
    pprint.pprint(unpacked)

    # Sending a reply to client
    UDPServerSocket.sendto(bytesToSend, address)