# -*- coding: utf-8 -*-
"""
Created on Mon Jul  4 17:11:53 2022

@author: David
"""
import AMazeThing_Keys as KEY
#SQL Things
sqlConn = None
cursor = None
sqlCursor = None

#Session MetaData 
Session = {KEY.SESSIONID:-1, "IP":"0.0.0.0","Started":-1,"Stopped":-1, KEY.ACCELBLOCKSIZE:-1,
           KEY.ACCELSAMPLETIME:-1,KEY.HALLBLOCKSIZE:-1, KEY.HALLSAMPLETIME:-1, KEY.HALLMASIZE:-1,
           KEY.HALLREPORTTIME:-1,"USEHZ":-1, KEY.NUMHALLSENSORS:-1, KEY.FWVER:"N/A", "BadPacketCount":0,
           KEY.STARTSESSION:-1,KEY.ENDSESSION:-1, "SQLConn":sqlConn, "SQLCursor":sqlCursor, "Closed":False}

Sessions = [Session]
print(Sessions)
Sessions.append(Session.copy())
Sessions[0]["IP"] = "192.168.1.1"
Sessions[1]["IP"] = "-7-7-7-7-7"


print(Sessions)