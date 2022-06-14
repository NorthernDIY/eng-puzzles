import socket, msgpack,time, sys

VERSIONNUM = 0.79
SERVICETXT = "MAZE_SERVER"

def broadCastService(port, interval):
    
    #Todo use a mutex and have it not available when a session is in progress
    #portB
    ServiceDictionary = {"SVC":SERVICETXT,"VER":VERSIONNUM,"PORT":1001}
    ServiceMessage = msgpack.dumps(ServiceDictionary,use_single_float=True,strict_types=True)
    #ServiceMessage = [131, 163, 83, 86, 67, 171, 77, 65, 90, 69, 95, 83, 69, 82, 86, 69, 82, 163, 86, 69, 82, 203, 63, 233, 71, 174, 20, 122, 225, 72, 164, 80, 79, 82, 84, 205, 30, 97]
    
    interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
    allips = [ip[-1][0] for ip in interfaces]
    while True:
        for ip in allips:
            UDPBROADCASTSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            UDPBROADCASTSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            UDPBROADCASTSocket.bind((ip,0))
            UDPBROADCASTSocket.sendto(ServiceMessage,("255.255.255.255", port))
            UDPBROADCASTSocket.close()
            print("BRDCAST", flush = True)
        time.sleep(10)


if __name__== '__main__':
    try:
        args=sys.argv
        alen=len(args)
        if alen==5:
            for item in range(1,alen,2):
                if args[item] == "--interval":
                    interval = args[item +1]
                if args[item] == "--port":
                    port = int(args[item +1])
            print("Starting Service Broadcast (All Interfaces) every %d Seconds on Port: %d" %(interval,port))
            broadCastService(port, interval)
        else:
            print("Starting Service Broadcast every 10 Seconds on Port: 7777")
            broadCastService(7777, 10)
    except KeyboardInterrupt:
        print("Broadcast halted")
        sys.exit()