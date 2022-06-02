import json
import random
import requests
import sys
import socket
import msgpack
from datetime import datetime, timedelta

#random.seed(1)  # comment out this line to have real random data


serverAddressPort = ("127.0.0.1", 20002)
bufferSize = 2000

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.settimeout(4)

start_time = datetime.now()

# returns the elapsed milliseconds since the start of the program


def millis():
    dt = datetime.now() - start_time
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
        1000 + dt.microseconds / 1000.0
    return ms


hallBlockSize = 8  # number of hall samples contained in a packet
accelBlockSize = 50  # number of accelerometer samples contained in a packet

sampleIntervalHall = 50  # on the ESP32 we sample the AVERAGE every 50mSec
sampleIntervalAccel = 10  # on the ESP32 we sample every 10mSec

# Hall packet transmission interval in milliseconds
reportIntervalHall = sampleIntervalHall * hallBlockSize
# Accell Packet transmission interval in milliseconds
reportIntervalAccel = sampleIntervalAccel * accelBlockSize

NumSensors = 24  # Number of Hall sensors to simulate
# Time stamp each hall packet (unit is in reportIntervalHall mSec)
timestamp_hall = 0
# Time stamp each accel packet (unit is in interval_accel mSec)
timestamp_accel = 0

# blank dictionaries for storing data -- These get converted from python dictionary to msgpack
HallDictionary = {}
AccelDictionary = {"T": timestamp_accel, "x": [], "y": [], "z": []}
ResetDictionary = {"Hello": 0.79,
                   "NumSens": NumSensors,
                   # Hall Sensor Stuff
                   "Hbs": hallBlockSize,
                   "H_Samp_Int": sampleIntervalHall,
                   # Accelerometer Stuff
                   "Abs": accelBlockSize,
                   "A_Samp_Int": sampleIntervalAccel}
print(msgpack.dumps(ResetDictionary))
#resetPost = requests.post(Reset_url, json.dumps(JsonRDictionary), headers = jsonHeaders)
msgFromClient = msgpack.dumps(ResetDictionary)
bytesToSend = msgFromClient
print("*MSGPACK* Sending: %d bytes...\n" % (len(bytesToSend)))
print("(*JSON* Would be: %d bytes)" % (len(json.dumps(ResetDictionary))))
try:
	UDPClientSocket.sendto(bytesToSend, serverAddressPort)
	SessionIDResponse = UDPClientSocket.recvfrom(bufferSize)[0]
	print(SessionIDResponse)
	SessionID = msgpack.loads(SessionIDResponse)["ID"]
	print("Session ID: %d" %(SessionID))
except socket.timeout:
	print("TIMEOUT! No response from Server, Exiting.")
	UDPClientSocket.close()
	sys.exit()
# sys.exit()
runTime = millis()
halltime = runTime  # tracks last packet send time
acceltime = runTime  # tracks last packet send time
while timestamp_hall < 60*60 / 50e-3:  # 1 Hour of 50mSec Packets
	try:
		#print(runTime - halltime)

		# print((runTime-halltime)>reportIntervalHall)
		# Hall Sensor Packet
		if ((runTime - halltime) > (reportIntervalHall)):
			halltime = runTime
			HallDictionary['TH'] = timestamp_hall
			for index in range(0, hallBlockSize):
				KeyValx = "x" + str(index)
				KeyValy = "y" + str(index)
				#KeyValz = "Hz" + str(index)
				HallDictionary[KeyValx] = []
				HallDictionary[KeyValy] = []
				#JsonHDictionary[KeyValz] = []

				for x in range(0, NumSensors):
					HallDictionary[KeyValx].append(random.randrange(-2047, 2047))
					HallDictionary[KeyValy].append(random.randrange(-2047, 2047))
					# JsonHDictionary[KeyValx].append(-2046.012)
					# JsonHDictionary[KeyValy].append(-2046.012)
					#JsonHDictionary[KeyValz].append(random.randrange(-2047, 2047))

			if (reportIntervalHall > 500):
				print("Hall Packet: ", end="")
				# print(JsonHDictionary)
				# print()

			jsonHObject = json.dumps(HallDictionary)
			#r = requests.post(Hall_url, json = jsonHObject, headers=jsonHeaders)
			msgFromClientH = msgpack.dumps(HallDictionary)
			bytesToSend = msgFromClientH
			print("H  *MSGPACK* Sending: %d bytes..." % (len(bytesToSend)), end="")
			print("(*JSON* Would be: %d bytes)" %
				  (len(json.dumps(HallDictionary))))
			UDPClientSocket.sendto(bytesToSend, serverAddressPort)
			timestamp_hall = timestamp_hall + (hallBlockSize*sampleIntervalHall)
			HallDictionary = {}

		# Accel Packet
		if ((runTime - acceltime)>(reportIntervalAccel)):
			# if (0):
			acceltime = runTime

			HallDictionary["TA"] = timestamp_accel
			for x in range(0, accelBlockSize):
				AccelDictionary["x"].append(
					random.randrange(-2047, 2047))
				AccelDictionary["y"].append(
					random.randrange(-2047, 2047))
				AccelDictionary["z"].append(
					random.randrange(-2047, 2047))

			"""if (reportIntervalAccel > 500):
				print("Accel Packet: ", end="")
				print(AccelDictionary)
				print()
			"""
			jsonAObject = json.dumps(AccelDictionary)
			#r = requests.post(Accel_url, json=jsonAObject, headers=jsonHeaders)
			msgFromClientA = msgpack.dumps(AccelDictionary)
			bytesToSend = msgFromClientA
			print("A  *MSGPACK* Sending: %d bytes..." % (len(bytesToSend)), end="")
			print("(*JSON* Would be: %d bytes)" %
				  (len(json.dumps(AccelDictionary))))
			UDPClientSocket.sendto(bytesToSend, serverAddressPort)
			timestamp_accel = timestamp_accel + \
				(accelBlockSize*sampleIntervalAccel)
			AccelDictionary = {"TA": timestamp_accel, "x": [], "y": [], "z": []}

		# Update current time for next loop
		runTime = millis()
	except socket.timeout:
		print("TIMEOUT! No response from Server, Exiting.")
		UDPClientSocket.close()
		sys.exit()
