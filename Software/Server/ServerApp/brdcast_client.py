# -*- coding: utf-8 -*-
"""
Created on Fri Jun 10 22:23:22 2022

@author: David
"""

import socket
import msgpack

trys = 0
while(1):
        serverResponse = None        
        UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        #UDPClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        #UDPSocket.settimeout(4)
        UDPServerSocket.bind(('',7777))
        decoded = None
        serverResponse = UDPServerSocket.recvfrom(2000)
        if serverResponse != None:
            
            print("\nNew BroadCast MSG from: ",end="")
            print(serverResponse[1])
            print("Size: %d   Payload: %s "%(len(serverResponse[0]),serverResponse[0]))
            print("\tTotal Size: (With Overhead +50B) ~: %d"%(len(serverResponse[0])+50))
            decoded = msgpack.unpackb(serverResponse[0])
            print("\t\tService Offered %s" %decoded["SVC"])
            print("\t\tVersion %s" %decoded["VER"])
            print("\t\tPort %s" %decoded["PORT"])
            