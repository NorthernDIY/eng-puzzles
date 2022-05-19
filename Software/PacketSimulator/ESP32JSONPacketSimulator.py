import json,random,requests, sys, socket, msgpack
from datetime import datetime,timedelta

random.seed(1)#comment out this line to have real random data


serverAddressPort   = ("127.0.0.1", 20002)
bufferSize          = 1024

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


start_time = datetime.now()

# returns the elapsed milliseconds since the start of the program
def millis():
   dt = datetime.now() - start_time
   ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
   return ms

hallBlockSize = 5 #number of hall samples contained in a packet
accelBlockSize = 5 #number of accelerometer samples contained in a packet

sampleIntervalHall = 500 #on the ESP32 we sample the AVERAGE every 50mSec
sampleIntervalAccel = 1000 # on the ESP32 we sample every 10mSec

reportIntervalHall = sampleIntervalHall * hallBlockSize #Hall packet transmission interval in milliseconds
reportIntervalAccel = sampleIntervalAccel * accelBlockSize#Accell Packet transmission interval in milliseconds

NumSensors = 4# Number of Hall sensors to simulate
timestamp_hall = 0;  #Time stamp each hall packet (unit is in reportIntervalHall mSec)
timestamp_accel = 0; #Time stamp each accel packet (unit is in interval_accel mSec)

Hall_url = 'http://192.168.1.51:5000/h'#flask application endpoint url for hall packet
Accel_url = 'http://192.168.1.51:5000/a'#flask application endpoint url for accel packet
Reset_url = "http://192.168.1.51:5000/r"
jsonHeaders = {'content-type': 'application/json'}

#blank dictionaries for storing data -- These get converted from python dictionary to json
JsonHDictionary = {}
JsonADictionary = {"T":timestamp_accel,"Ax":[],"Ay":[],"Az":[]}
JsonRDictionary = {"Version":"Sim v0.9",
                   "NumSens":NumSensors,
                   #Hall Sensor Stuff
                   "Hbs":hallBlockSize,
                   "H_Samp_Int":sampleIntervalHall,
                   #Accelerometer Stuff
                   "Abs":accelBlockSize,
                   "A_Samp_Int": sampleIntervalAccel}
print(msgpack.dumps(JsonRDictionary))
#resetPost = requests.post(Reset_url, json.dumps(JsonRDictionary), headers = jsonHeaders)
msgFromClient       = msgpack.dumps(JsonRDictionary)
bytesToSend         = msgFromClient
print("MSGPACK Sending: %d bytes...\n"%(len(bytesToSend)))
print("(JSON Compared to : %d bytes)"%(len(json.dumps(JsonRDictionary))))
UDPClientSocket.sendto(bytesToSend, serverAddressPort)
SessionID = UDPClientSocket.recvfrom(bufferSize)
print(SessionID)
sys.exit()
runTime = millis()
halltime = runTime #tracks last packet send time 
acceltime = runTime#tracks last packet send time
while timestamp_hall <60*60 /50e-3:#1 Hour of 50mSec Packets

    #print(runTime - halltime)
    
    #print((runTime-halltime)>reportIntervalHall)
    #Hall Sensor Packet
    if ((runTime - halltime)>(reportIntervalHall)):
        halltime = runTime
        JsonHDictionary['T'] = timestamp_hall        
        for index in range(0,hallBlockSize):
            KeyValx = "Hx" + str(index)
            KeyValy = "Hy" + str(index)
            KeyValz = "Hz" + str(index)
            JsonHDictionary[KeyValx] = []
            JsonHDictionary[KeyValy] = []
            JsonHDictionary[KeyValz] = []
                
            for x in range(0, NumSensors):
                JsonHDictionary[KeyValx].append(random.randrange(-2047000, 2047000)/1000.0000)
                JsonHDictionary[KeyValy].append(random.randrange(-2047000, 2047000)/1000.0000)
                JsonHDictionary[KeyValz].append(random.randrange(-2047000, 2047000)/1000.0000)
    
        

        if (reportIntervalHall >500):
            print("Hall Packet: ", end = "")
            print(JsonHDictionary)
            print()
    
        jsonHObject = json.dumps(JsonHDictionary)
        r = requests.post(Hall_url, json = jsonHObject, headers=jsonHeaders)
        timestamp_hall = timestamp_hall + (hallBlockSize*sampleIntervalHall)
        JsonHDictionary = {}

    #Accel Packet
    if ((runTime - acceltime)>(reportIntervalAccel)):
    #if (0):
        acceltime = runTime

        JsonHDictionary["T"] = timestamp_accel
        for x in range(0, accelBlockSize):
            JsonADictionary["Ax"].append(random.randrange(-2047000, 2047000)/1000.0000)
            JsonADictionary["Ay"].append(random.randrange(-2047000, 2047000)/1000.0000)
            JsonADictionary["Az"].append(random.randrange(-2047000, 2047000)/1000.0000)
    
        if (reportIntervalAccel>500):
            print("Accel Packet: ", end = "")
            print(JsonADictionary)
            print()
    
        jsonAObject = json.dumps(JsonADictionary)
        r = requests.post(Accel_url, json = jsonAObject, headers = jsonHeaders)
        timestamp_accel = timestamp_accel + (accelBlockSize*sampleIntervalAccel)
        JsonADictionary = {"T":timestamp_accel,"Ax":[],"Ay":[],"Az":[]}
    
    #Update current time for next loop
    runTime = millis()




#Ignore below, was used in packet size estimator but may be useful on ESP32 if
#we want to build our packets in a bruteforce way without Json library
"""#Brute force way to do this in python
for x in range(16, 40):
    #print("{'" + hex(x) +"':", end = "")
    packetBuffer.write(str(random.randrange(-2047000, 2047000)/1000.0000))
    if (not (x-3)% 4==0):
        packetBuffer.write(",")
    elif(x<39) :
        packetBuffer.write(",\n")

packetBuffer.write("],\n\"Y\":[")
for y in range(16, 40):
    #print("{'" + hex(x) +"':", end = "")
    packetBuffer.write(str(random.randrange(-2047000, 2047000)/1000.0000))
    if (not (y-3)% 4==0):
        packetBuffer.write(",")
    elif (y<39) :
        packetBuffer.write(",\n")
packetBuffer.write("],\n\"Z\":[")
for z in range(16, 40):
    #print("{'" + hex(x) +"':", end = "")
    packetBuffer.write(str(random.randrange(-2047000, 2047000)/1000.0000))
    if (not (z-3)% 4==0):
        packetBuffer.write(",")
    elif(z<39) :
        packetBuffer.write(",\n")
packetBuffer.write("]\n}\n}")"""
