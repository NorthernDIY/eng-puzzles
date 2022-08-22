# -*- coding: utf-8 -*-
"""
Created on Mon Jul  4 16:11:15 2022

@author: David
"""


#//MSGPACK (JSONDOC) KEYS
HALLTIME ="TH"  #Presence of key when sent to server indicates Hall packet (this is the time stamp)
ACCELTIME = "TA" #Presence of key when sent to server indicates Accel packet (this is the time stamp)
HALLDATA = "D"
ENDSESSION = "Bye" #Sent to server to close session
STARTSESSION = "Hey" #Sent to server to start a new session
FWVER = "FWV"

#Deprecated Tags
"""
BIOSAVX = "BSX" #Run time calibration constant
BIOSAVY = "BSY" #Run time calibration constant
BIOSAVZ = "BSZ" #Run time calibration constant
HCALX ="H_CAL_X" #Run time baseline noise data
HCALY ="H_CAL_Y"#Run time baseline noise data sent at start of session
HCALZ ="H_CAL_Z"#Run time baseline noise data sent at start of session
"""

BSD = "B" # Combined 3x BSX,BSY,BSZ tag

HN_1 = "N0" #Hall Sensor Noise Snapshot 1
HN_2 = "N1" #Hall Sensor Noise Snapshot 2
HN_3 = "N2" #Hall Sensor Noise Snapshot 3
SRVCHECK ="CHK"#Periodic message sent with this queries server to ensure its still active
SRVOK ="OK"#Response from server expected from SRVCHECK
SESSIONID ="ID"#This key is the session id
SESSIONPORT = "PORT"
REALSTART ="S"#Micros() of packet sampling start
REALSTOP ="E"#Micros() of packet sampling end

NUMHALLSENSORS ="NumSens"#Number of sensors on maze unit
HALLBLOCKSIZE ="HBS" #Block size of Hall sensor data
HALLHASZ ="HallHasZ" #Tell server we can provide Hall Z data if desired
ACCELBLOCKSIZE ="ABS" #Block size of Accel sensor data

#Server sets these parameters during session start
SENDHZ ="WantHZ" #Server session start message will send this key with 0 = no Z, 1 = send Z
SENDSTS ="STS" #Server session start reply, 1 = send sample start/end times, 0 = don't send
ACCELSAMPLETIME ="AST" #Session start reply sends this key with sample time in ms for Accelerometer
HALLSAMPLETIME ="HST"  #Session start reply sends this key with sample time in ms for Hall sensor matrix
HALLMASIZE ="HMS" #Session start reply sends this key with # of samples in ring buffer used for moving average of hall sensors
HALLREPORTTIME ="HRT" #Session start reply sends this key with how often we report the hall sensor MAfiltered output
