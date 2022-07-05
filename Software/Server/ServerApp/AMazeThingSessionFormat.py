# -*- coding: utf-8 -*-
"""
Created on Mon Jul  4 20:07:54 2022

@author: David
"""
import AMazeThing_Keys as KEY
LHS = "LASTHALLSTART"
LAS = "LASTACCELSTART"
IP = "IP"
ST = "Started"
ET = "Stopped"
HZ = "USEDHZ"
BPC = "BadPacketCount"
SQP = "SQLConn" #SQL 'Pipe'
SQC = "SQLCursor"
SC = "Closed"
ID = KEY.SESSIONID
ABS = KEY.ACCELBLOCKSIZE
AST = KEY.ACCELSAMPLETIME
HBS = KEY.HALLBLOCKSIZE
HST = KEY.HALLSAMPLETIME
HMA = KEY.HALLMASIZE
HRT = KEY.HALLREPORTTIME
NHS = KEY.NUMHALLSENSORS
FWV = KEY.FWVER
SS = KEY.STARTSESSION
ES = KEY.ENDSESSION
LDT = "LASTDATATIME"
#Session MetaData
sessionObject = {ID:-1, IP:"0.0.0.0",ST:-1,ET:-1, ABS:-1,
           AST:-1,HBS:-1, HST:-1, HMA:-1,
           HRT:-1,HZ:-1, NHS:-1, FWV:"N/A", BPC:0,
           SS:-1,ES:-1, SQP:None, SQC:None, SC:False,
           LHS:-1, LAS:-1, LDT:-1}

Sessions = []