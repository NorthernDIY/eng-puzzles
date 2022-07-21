#DEFAULT SETTINGS (Don't change these defaults, edit the config file or use the command line arguments to adjust them)
DEF_IP = "127.0.0.2"        #IP address to bind services to
DEF_BCAST_PORT = 1998       #UDP Broadcast port to announce server availability (Direction: SERVER -> ESP32)
DEF_BCAST_INT = 5           #UDP Broadcast interval (seconds) to announce server availability (Direction: SERVER -> ESP32)
DEF_START_PORT = 1999       #UDP Port of Session Initiator Thread (Direction: ESP32->SERVER->ESP32)
DEF_PORTMIN = 12000         #UDP Port lower limit for active sessions
DEF_PORTMAX = 15000         #UDP Port upper limit for active sessions
DEF_RAWDIR = "RawDat/"      #Directory to store Raw Session Database files in
DEF_REPORTDIR = "Reports/"  #Directory to store Raw Session Database files in
DEF_SDB_PREFIX = 'MZRaw_S'    #FileName prefix for Raw database files
DEF_SRP_PREFIX = 'Report_S'   #FileName prefix for Report files
DEF_KEEP_PKT_TIMES = 1      #1 = Record Sampling Start/End times into the Database (based on ESP32 millis())
DEF_GET_HZ = 1              #1 = Ask for (and store in DB) Hall Z data                                                                                      (SESSION PARAMETER)
DEF_GET_SAMPLETIMES = 1         #1 = Ask for ESP32 to send Block Sample period Start/End times(based on ESP32 millis()) **Not saved unless KEEP_PKT_TIMES = 1   (SESSION PARAMETER)
DEF_HST = 10                #Hall Matrix Sample Period in ms.  Determines how often the Moving Average Buffer is updated. Do not use values <5ms!!!         (SESSION PARAMETER)
DEF_HRT = 100               #Hall Matrix Report Period in ms.  Determines how often the MA Buffer is sampled for a data point.  Do not use values <5ms      (SESSION PARAMETER) 
DEF_HMA = 20                #Hall Matrix Moving Average size.  Do not use Values > 60                                                                       (SESSION PARAMETER)
DEF_AST = 40                #Accelerometer Sample Period in ms.                                                                                             (SESSION PARAMETER)
DEF_SESSIONID = 0           #Session ID, No reason to start at anything but 0 -OR- the value stored in the CFG file
DEF_DISP_PKT_RX = 0         #1 = Print a message whenever a packet has been recieved, includes size of message in bytes
DEF_BUFFERSIZE  = 10000     #Maximum UDP packet size (ESP32 doesn't send anything larget than 1500 bytes so this is way overkill

DEF_RESET_AT_RUN = 0        #Clear all DB Files and Reports and Reset to defaults every Run (DEBUG USE)

#DBG_DEFIP = "192.168.137.1" #Default IP when RESET_AT_RUN is enabled
#DBG_DEFIP = "10.42.0.1"

#DICTIONARY KEYS
K_SID = "LastSessionID"
K_BCP = "BroadCastPort"
K_BCI = "BroadcastInterval"
K_STP = "InitiatorPort"
K_SPL = "SessionPortMin"
K_SPU = "SessionPortMax"
K_GHZ = "GetHallZ"
K_GST = "GetSampleTimes"
K_HST = "HallSamplePeriod"
K_HRT = "HallReportPeriod"
K_HMA = "HallMovingAverageSize"
K_AST = "AccelSampleTime"
K_KPT = "KeepPacketTimes"
K_DBRD = "DBRawDirectory"
K_REPD = "ReportDirectory"
K_SDPFX = "SessionDBPrefix"
K_SRPFX = "SessionReportPrefix"
K_RAR  = "ResetAtRun"
K_DPR = "DisplayPacketRx"
K_IP = "LocalIP"
K_BUFS ="IncomingBufferSize"

#Command line argument mapping stuff

C_BCP = "-bp"
C_BCI = "-bi"
C_STP = "-sp"
C_SPU = "-pmax"
C_SPL = "-pmin"
C_HST = "-hst"
C_HRT = "-hrt"
C_GST = "-gst"
C_GHZ = "-ghz"
C_HMA = "-hma"
C_AST = "-ast"
C_KPT = "-kpt"
C_RAR = "-rar"
C_DPR = "-dpr"
C_IP = "-ip"
CMD_ARGS = {C_BCP:K_BCP,C_BCI:K_BCI,C_STP:K_STP,C_SPU:K_SPU,C_SPU:K_SPL,
            C_HST:K_HST,C_HRT:K_HRT,C_GST:K_GST,C_GHZ:K_GHZ,C_HMA:K_HMA,
            C_AST:K_AST,C_KPT:K_KPT,C_RAR:K_RAR, C_DPR:K_DPR,C_IP:K_IP}
CMD_ARGS_TYPES = {C_BCP:"INT",C_BCI:"INT",C_STP:"INT",C_SPU:"INT",C_SPU:"INT",
            C_HST:"INT",C_HRT:"INT",C_GST:"INT",C_GHZ:"INT",C_HMA:"INT",
            C_AST:"INT",C_AST:"INT",C_KPT:"INT",C_RAR:"INT", C_DPR:"INT",C_IP:"TEXT"}
CMD_ARGS_UPDATEMESSAGES={C_BCP:"Service Broadcast Port", C_BCI:"Service Broadcast Interval",
                         C_STP:"Session Initiator Port", C_SPU:"Max session Port", C_SPL:"Min Session Port",
                         C_HST:"Hall Sample Period",C_AST:"Acclerometer Sample Period",
                          C_HRT:"Hall Report Period", C_GST:"Send Sample Times",
                         C_GHZ:"Send Hall Z Data", C_HMA:"Hall Moving Average Buffer Size", C_KPT:"Keep Packet Times",
                         C_RAR:"Reset at Runtime", C_DPR:"Display Packet Rx", C_IP:"Server IP address to Bind"}


_KEYS = [K_SID,K_BCP, K_BCI, K_STP, K_SPL, K_SPU, K_GHZ, K_GST, K_HST, K_HRT, K_HMA,
            K_AST, K_KPT, K_DBRD, K_REPD, K_SDPFX, K_SRPFX, K_RAR, K_DPR, K_IP, K_BUFS]

_DEFAULTS = {K_SID:DEF_SESSIONID,K_BCP:DEF_BCAST_PORT, K_BCI:DEF_BCAST_INT, K_STP:DEF_START_PORT,
           K_SPL:DEF_PORTMIN, K_SPU:DEF_PORTMAX, K_GHZ:DEF_GET_HZ, K_GST:DEF_GET_SAMPLETIMES,
           K_HST:DEF_HST, K_HRT:DEF_HRT, K_HMA:DEF_HMA, K_AST:DEF_AST, K_KPT:DEF_KEEP_PKT_TIMES,
           K_DBRD:DEF_RAWDIR, K_REPD:DEF_REPORTDIR, K_SDPFX:DEF_SDB_PREFIX,
           K_SRPFX:DEF_SRP_PREFIX, K_RAR:DEF_RESET_AT_RUN, K_DPR:DEF_DISP_PKT_RX,
           K_IP:DEF_IP, K_BUFS:DEF_BUFFERSIZE}