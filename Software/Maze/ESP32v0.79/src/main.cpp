/*
 *  Portions of this code are bsed on "Example code for reading an ALS31313 3D Hall Effect Sensor on ESP32 in Arduino environment." by VintLabs 
 *  which in tern is based on code provided by Allegro Microsystems:
 *      Example source code for an Arduino to show how to communicate with an Allegro ALS31300
 *          Written by K. Robert Bate, Allegro MicroSystems, LLC.*    
 *    
 *    ECE4600 G15 Modified by David Stewart Oct 26/2021
 *      Added Power control stuff
 *    Modified by David Stewart Nov 11/2021
 *      Major rewrite of ReadADC function to accomodate use of Structs to hold sensor and moving average data.  
 *      Addition of sensor init routine
 *      Added independent report period interval, in addition to existing sample period interval.
 *      Expanded # sensor read up to 5
 */

//Board:  ESP32 Nano32

//To speed up final project computational speed, uncomment the following!
//This will require additional Flash/Ram, but can provide substantial speed increases.
#pragma GCC optimize ("-O2")

//Libraries required
#include <Arduino.h>
#include <Wire.h> //Provides I2C
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h> //Data sent is serialized in MSGPACK, this library gives us easy access to that data type
#include "SparkFun_LIS2DH12.h"//Accelerometer for tremor identification
//#include <esp_task_wdt.h>
#include <esp_wifi.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define VERSIONSTRING "v0.85" //Firmware Version String
//WiFi credentials
#define G15_SSID "AMazeThing"
#define G15_PASSWORD "hn7P8Egh"

#define MSGBUFFERSIZE 1500 //Size of char buffers to serialze msgpack data into (2x buffers)

#define SERVERCHECKTIME 3000 //Check server alive every nnnn/1000 seconds
#define HOME_SWITCH_ACTIVATION_TIME 300 //Time to wait after inital switch open before starting a new session (Part of debounce)
#define SPINNERTASKDELAY 500 //Time between spins for the user feedback spinner task

#define DBGSERIAL 0 //Output various information over serial  (Used during parameter tuning)
#define INITSERIAL 0
//Hall Sensor Matrix Properties
//#define HALLAVERAGINGSAMPLES 20 //Number of samples in moving average
#define HALLAVERAGINGSAMPLES_MAX 60  //Max size of moving averange ring buffer
#define NUMSENSORS 24 //Should be 24 in final project

//Once total msgpack size exceeds about 1470 bytes, packets are fragmented and then the system breaks!
#define BLOCKSIZEHALL 15 //17 is about the maximum we can go before fragmentation will start to become an issue - MSGBUFFERSIZE (based on MTU) is the limitation else we start spanning packets

uint16_t HALLAVERAGINGSAMPLES = 20;
uint16_t HALLSAMPPERIOD = 25;
uint16_t HALLREPORTTIME = 200;
uint16_t ACCELSAMPPERIOD = 40;
//Accelerometer Properties
//#define ACCELSAMPPERIOD 40// 1/(40/1000) = 25 Hz Sample Rate --> Nyquist 20Hz
#define BLOCKSIZEACCEL 150

//ALS31313 Hall Sensor parameters (from datasheet)
#define EEPROMDEFAULT     0b00000000000000000000001111100000
#define EEPROMADDRESSMASK 0b11111111111111100000001111111111
#define CAM_REGISTER 0x35
//#define CAM_KEY 0x2C41354 //Was incorrect in example code //ALS31313 Sensors
#define CAM_KEY 0x2C413534 //ALS31300 Sensors

//Server Properties to look for (and what port to listen for service broadcast
#define MINSERVERVER 0.79
#define SERVICEREQUIRED "MAZE_SERVER"
#define SERVICEBROADCASTPORT 7777
//Keys used in service broadcast message
#define KEY_SERVICETYPE "SVC"
#define KEY_SERVICEVER "VER"
#define KEY_SERVICEPORT "PORT"

//MSGPACK (JSONDOC) KEYS
#define KEY_HALLTIME "TH" //Presence of key when sent to server indicates Hall packet (this is the time stamp)
#define KEY_HALLDATA "D"
#define KEY_ACCELTIME "TA"//Presence of key when sent to server indicates Accel packet (this is the time stamp)
#define KEY_ENDSESSION "Bye"//Sent to server to close session
#define KEY_STARTSESSION "Hey"//Sent to server to start a new session
#define KEY_FWVER "FWV"
#define KEY_BIOSAVX "BSX"//Run time calibration constant sent at start of session
#define KEY_BIOSAVY "BSY"//Run time calibration constant sent at start of session
#define KEY_BIOSAVZ "BSZ"//Run time calibration constant sent at start of session
#define KEY_HCALX "H_CAL_X"//Run time baseline noise data sent at start of session
#define KEY_HCALY "H_CAL_Y"//Run time baseline noise data sent at start of session
#define KEY_HCALZ "H_CAL_Z"//Run time baseline noise data sent at start of session
#define KEY_SRVCHECK "CHK"//Periodic message sent with this queries server to ensure its still active
#define KEY_SRVOK "OK"//Response from server expected from SRVCHECK
#define KEY_SESSIONID "ID"//This key is the session id
#define KEY_REALSTART "S"//Micros() of packet sampling start
#define KEY_REALSTOP "E"//Micros() of packet sampling end

#define KEY_NUMHALLSENSORS "NumSens"//Number of sensors on maze unit
#define KEY_HALLBLOCKSIZE "HBS" //Block size of Hall sensor data
#define KEY_HALLHASZ "HallHasZ" //Tell server we can provide Hall Z data if desired
#define KEY_ACCELBLOCKSIZE "ABS" //Block size of Accel sensor data

//Server sets these parameters during session start
#define KEY_SENDHZ "WantHZ" //Server session start message will send this key with 0 = no Z, 1 = send Z
#define KEY_SENDSTS "STS" //Server session start reply, 1 = send sample start/end times, 0 = don't send
#define KEY_ACCELSAMPLETIME "AST" //Session start reply sends this key with sample time in ms for Accelerometer
#define KEY_HALLSAMPLETIME "HST"  //Session start reply sends this key with sample time in ms for Hall sensor matrix
#define KEY_HALLMASIZE "HMS" //Session start reply sends this key with # of samples in ring buffer used for moving average of hall sensors
#define KEY_HALLREPORTTIME "HRT" //Session start reply sends this key with how often we report the hall sensor MAfiltered output
//see HALLAVERAGINGSAMPLES_MAX above

//I2C Device Addresses
#define OLED_I2C_ADDRESS 0x3C
#define ACCEL_ADDRESS 0x19

//This is the power up addresses of the sensors when using the voltage divider inputs to ADR0/1
//Repeated for convenience across 2 channels
const uint8_t FactoryPowerUpAddresses[NUMSENSORS] = {96,97,98,99,100,101,102,103,104,105,106,107,96,97,98,99,100,101,102,103,104,105,106,107};

//Addresses of hall sensors if they have been programmed
const uint8_t sensorAddresses[] = {12,22,11,21,14,24,13,23,16,26,15,25,32,42,31,41,34,44,33,43,36,46,35,45};
const uint8_t calSensorAddress = 31;
const uint8_t calSensorIndexLocation = 14;//Location in the array above...used to reference sensor data for BiotSavart Const Calculation

//Actual C code files C:\Users\---username---\.platformio\packages\framework-espidf\components\driver
//#include "driver/i2c.h" //Needed to allow use of I2C peripheral settings to actually reach 1MHz!
TwoWire I2C_HS = TwoWire(0);//HS I2C (Hall Sensors)
TwoWire I2C_MS = TwoWire(1);//MS I2C (Accelerometer)

SPARKFUN_LIS2DH12 accel;

const char* ssid = G15_SSID;
const char* password = G15_PASSWORD;

// Return values of endTransmission in the Wire library
#define kNOERROR 0
#define kDATATOOLONGERROR 1
#define kRECEIVEDNACKONADDRESSERROR 2
#define kRECEIVEDNACKONDATAERROR 3
#define kOTHERERROR 4

uint16_t mySession = 0; //Tracks session ID as returned by server at Hello
uint16_t lastSession = 0;//Track last session for display purposes
bool serverOK = false;// Periodically verify server alive-edness, store result here...

bool noDispChange = false;//Track when display is updated which determines if we refresh everything periodically or not.
bool SENDSTS = false;  //Set by server parameter, send Sample Start/End times
//Moving Average Struct to store Hall sensor readings
typedef struct{
  int16_t sampX[HALLAVERAGINGSAMPLES_MAX];
  int16_t sampY[HALLAVERAGINGSAMPLES_MAX];
  int16_t sampZ[HALLAVERAGINGSAMPLES_MAX];
  int16_t avgX,avgY,avgZ;
  int totX,totY,totZ;
  uint8_t address; 
  uint8_t sampIndex;
} MASensor;
MASensor sensors[NUMSENSORS];

enum SpinTaskStates{
  spin_H=0,
  spin_LS=1,
  spin_V1=2,
  spin_RS=3
};
enum SpinTaskStates SpinState;

enum ServerResponseStates{
  RESP_OK,
  RESP_BAD_DE,
  RESP_BAD_MK,
  RESP_TIMEOUT
};

//Number of matrix scans/Accelerometer reads to bundle into a packet
const uint8_t hallBlockSize = BLOCKSIZEHALL;
const uint8_t accelBlockSize = BLOCKSIZEACCEL;

//Json stores hallBlockSize sample periods worth of data.
StaticJsonDocument <(1250 * hallBlockSize)> hallDoc;//TUNING - Recalculate packet size based on new Hx# values
uint8_t hMsgBuffer[MSGBUFFERSIZE];//msgPack Buffer for Hall
JsonArray HallData;

JsonArray Ax,Ay,Az;

//These hold the powerup Hall Calibration data
float BiotSavartCalConstantX = 0.0;
float BiotSavartCalConstantY = 0.0;
float BiotSavartCalConstantZ = 0.0;

//Used for listening to service announce, checking, initiating and ending sessions with server
WiFiUDP wifiUDPclient;

uint16_t srvPort = 20002; //Default port is 20002, but this is actually set by service broadcast at startup
const uint16_t bcastPort = SERVICEBROADCASTPORT; //Port that device listens to service broadcasts on.
IPAddress srvAddress;

//Json stores accelBlockSize sample periods worth of data
StaticJsonDocument <(55*accelBlockSize)>accelDoc;//TUNING - Recalculate packet size based on new Ax# values
uint8_t aMsgBuffer[MSGBUFFERSIZE]; //msgPack Buffer for Accel
//Tracking timestamps of last sample, and last report.  TODO - CONSIDER Move to Timer interrupt based sampling


bool active = false;
bool sendHZ = true;

SemaphoreHandle_t hDocMutex, aDocMutex, hSampMutex;
SemaphoreHandle_t UDPMutexA, UDPMutexH; //Used to ensure sample and send tasks finish at session end
WiFiUDP aSendTaskUDP;
WiFiUDP hSendTaskUDP;

//PIN INFORMATION
//N-Channel disconnects ground from voltage divider circuit.  Causes ADR0/1 to float to VCC.  HIGH = Divider Enabled, LOW = Divider Disabled (Floats to VCC)
#define CTL_VDIVIDER 16

//P-Channel VCC3V3 High Side Switch for everything but the esp32.  Disconnects: VDivider, Vdivider Op Amp Buffer, Accelerometer, Hall Sensors.
#define _CTL_VCC3V3 4

//P-Channel high side switch for sensor channel power.  LOW = Sensor power enabled, HIGH = Sensor power disabled
#define _CTL_CHA1_PWR 17
#define _CTL_CHA2_PWR 18
#define _CTL_CHA3_PWR 19//Unused
#define _CTL_CHA4_PWR 5//Unused

//P-Channel high side switch for accelerometer power.  LOW = Sensor power enabled, HIGH = Sensor power disabled
#define _CTL_ACCEL_PWR 27

//Status LEDS
#define IND_LED1 33//Green
#define IND_LED2 25//Red
#define IND_LED3 26//Yellow

//Primary I2C Bus (1 MHz)
#define SCL_HS 22
#define SDA_HS 21

//Secondary I2C Bus (400 KHz)
#define SCL_MS 14
#define SDA_MS 12

//Reed Switch, lets us switch from waiting to runtime mode
#define REED_SWITCH 34

//Sensors interrupts, probably will be unused
#define ACCEL_INT 35
#define HALL_INT 23

//Function prototypes
//System Power Related
void powerOffSensors();
void powerOnSensors(bool useEEPROM);
void initPower();

//I/O Pin related
void initIO();
void initI2CBusses();
#define ledGreenOn() digitalWrite(IND_LED1, HIGH)
#define ledGreenOff() digitalWrite(IND_LED1, LOW)
#define ledRedOn() digitalWrite(IND_LED2, HIGH)
#define ledRedOff() digitalWrite(IND_LED2, LOW)
#define ledYellowOn() digitalWrite(IND_LED3, HIGH)
#define ledYellowOff() digitalWrite(IND_LED3, LOW)

//Hall Sensor Related
bool initHallSensor(uint8_t index);
void enterCam(uint8_t busAddress, uint32_t magicKey);
void IRAM_ATTR scanHallMatrix();
void IRAM_ATTR readALS31300ADC(uint8_t index);
uint16_t read(int busAddress, uint8_t address, uint32_t& value);
uint16_t write(int busAddress, uint8_t address, uint32_t value);

//Misc Hall Sensor
long IRAM_ATTR SignExtendBitfield(uint32_t data, int width);

//Usual Suspects
void setup();
void IRAM_ATTR loop();

//Calibration Related
void sendCalData();
void hallSensorZeroCal();

//Communications Related
void beginSession();
void endSession();
void IRAM_ATTR sampleHallTask( void * pvParameters);
void IRAM_ATTR reportHallTask(void * pvParameters);
void IRAM_ATTR pktzAndSendHallTask(void * pvParameters);

void IRAM_ATTR pktzAndSendAccelTask(void * pvParameters);
void IRAM_ATTR sampleAccelTask( void * pvParameters);

void newSessionConnect();
bool findServer(bool updateDisplay);
void CheckReady();
void CheckServerAlive(bool useDisp,bool findServerIfLost);
//void serverCheckInTask(void * pvParameters);

//User Interface Related
void spinnerTask( void * pvParameters);
void waitStylusHome(bool animate);


SSD1306AsciiWire oled(I2C_MS);

// setup
//
// Initializes the Wire library for I2C communications,
// Serial for displaying the results and error messages,
// the hardware and variables to blink the LED,
// and sets the ALS31300 into customer access mode.
void setup()
{
  setCpuFrequencyMhz(40); //Slow speed at startup to help reduce brownout issues
  ledGreenOn();
  ledYellowOn();
  ledRedOn();
  //esp_task_wdt_init(30, false); //set wdt to higher timeout
  
  //Setup pin directions
  initIO();  
  
  //Turn on main power, Hall Channels 1 & 2 and Accelerometer/Oled
  //Divider off = assume Hall eeproms are setup already (Addresses)
  initPower();
  ledGreenOff();
  ledRedOff();
  ledYellowOn();

  hDocMutex = xSemaphoreCreateMutex();//Used to prevent hall data from being reset before serialized
  aDocMutex = xSemaphoreCreateMutex();//Used to prevent accel data from being reset before serialized
  UDPMutexA = xSemaphoreCreateMutex();//Used to ensure sample/send tasks are stopped before closing session
  UDPMutexH = xSemaphoreCreateMutex();//Used to ensure sample/send tasks are stopped before closing session
  hSampMutex = xSemaphoreCreateMutex();
  
  initI2CBusses(); // Initialize the I2C communication ports
  delay(50);
  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
  oled.setContrast(127);
  oled.setFont(font5x7);
  oled.clear();
  oled.set2X();

  oled.print("AMazeThing");
  oled.set1X();
  oled.setCursor(45,2);
  oled.print(VERSIONSTRING);

  setCpuFrequencyMhz(240);
  delay(10);

  if (INITSERIAL)Serial.begin(230400);  

  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  oled.setCursor(0,3);
  oled.print("WiFi: ");
  while(WiFi.status() != WL_CONNECTED){
    delay(250);
  }
  oled.print(WiFi.localIP());
  oled.setCursor(0,4);
  oled.print("Hall: ");

  //Initialize each Hall sensor -- Enters CAM mode and sets up Full Loop Register
  bool okSensors = true;
  for (uint8_t index = 0; index<NUMSENSORS; index++){
    if (!initHallSensor(index)){
      okSensors = false;
    }
  }
  if (okSensors){
    oled.print("OK");
  }else{
    oled.print("InitError");
  }
  
  okSensors = true;
  oled.setCursor(63,4);
  oled.print("Accel: ");
  //Initialize Accelerometer
  accel.begin(ACCEL_ADDRESS, I2C_MS);
  accel.setDataRate(LIS2DH12_ODR_400Hz);
  if (accel.isConnected()){
    oled.println("OK");
  }else{
    oled.println("InitError");
  }
  //accel.setSensitivity();
  
  delay(1000);
  hallSensorZeroCal();
  delay(1000); 
  bool server_found = findServer(true);//This never returns unless a service announce has been heard.
  wifiUDPclient.begin(srvPort);//Start watching server comms port
  
  if (DBGSERIAL)Serial.println("AMazeThing...\nReady state starts in 1 Sec");
  delay(1000);
  oled.setContrast(31);
  CheckReady();
}

void CheckReady(){
  bool wifiOK = (WiFi.status() == WL_CONNECTED);
  CheckServerAlive(true,true);
  
  if (serverOK && wifiOK){
    if (!noDispChange){
      oled.clear();
      ledGreenOn();
      ledYellowOff();
      ledRedOff();
      oled.set2X();
      oled.print("  Ready!");
      oled.set1X();
      oled.setCursor(0,7);
      if (lastSession >0)
      {
        oled.print("Last Session: #");
        oled.print(lastSession,DEC);
      }
      noDispChange=true;
    }    
  }else{
    oled.clear();
    noDispChange=false;
    ledGreenOff();
    ledYellowOn();
    ledRedOff();
    bool wifiReset = false;
    if (WiFi.status()!= WL_CONNECTED){
      ledRedOn();
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      oled.setCursor(0,7);
      oled.print("WiFi: ");
      while(WiFi.status() != WL_CONNECTED){
        delay(200);
        //Serial.print(".");
      }
      oled.setCursor(6,7);
      oled.print("Re-Connected");
      wifiReset = true;
    }
    if (wifiReset)delay(1000);
    ledRedOff();
    oled.setCursor(0,0);
    oled.set2X();
    oled.println(" Checking");
    oled.println("  Server!");
    oled.set1X();
    oled.setCursor(0,6);
    oled.println("  Listening...");
    findServer(false);
    oled.setCursor(0,6);
    oled.clearToEOL();
    oled.println("  Found!");
    wifiUDPclient.begin(srvPort);//Start watching server specified port
  }
}
void CheckServerAlive(bool useDisp, bool findServerIfLost){
  serverOK = false;
  bool foundServer;
  if(useDisp){
    oled.set1X();
    oled.setCursor(120,7);
    oled.print(".");
  }
  if ((uint32_t)srvAddress != 0){
    wifiUDPclient.begin(srvPort);
    int trials = 0;
    while(!serverOK && trials<30){
      StaticJsonDocument <100> chkDoc;
      chkDoc[KEY_SRVCHECK] = 1;
      uint8_t buffer[100];
      uint8_t msgLen = serializeMsgPack(chkDoc,buffer);
      wifiUDPclient.beginPacket(srvAddress,srvPort);
      wifiUDPclient.write(buffer,msgLen);
      wifiUDPclient.endPacket();
      delay(100);
      StaticJsonDocument<100> serverResponse;
      int pSize = wifiUDPclient.parsePacket();
      if (pSize){
        int len = wifiUDPclient.read(buffer,100);
        DeserializationError de_error = deserializeMsgPack(serverResponse, buffer);
        if (de_error){
          if (useDisp){
            oled.print("Bad packet Rx'd");
            noDispChange=false;
          }
        }else if (serverResponse.containsKey(KEY_SRVOK))serverOK = true;
      }else{
        trials++;
      }
    serverResponse.clear();
    chkDoc.clear();
    }
    if (trials>30){
      if (useDisp){
        oled.clear();
        noDispChange=false;
        oled.set2X();
        oled.println(" Server\n  Lost");
        oled.println("Listening");
      }
      if (findServerIfLost) foundServer= findServer(false);
    }
  }
  //clear out any packets in the rx buffer
  while(wifiUDPclient.parsePacket()){
    uint8_t jnkBuffer[100];
    int len = wifiUDPclient.read(jnkBuffer,100);
  }
  if(useDisp){
    oled.setCursor(120,7);
    oled.clearToEOL();
  }
}

//Main loop just waits for reed switch to activate
void loop()
{
  uint32_t tTime = millis();
  uint32_t lastCheckin = 0;
  bool Triggered = false;
  while(1){
    if ((millis()-lastCheckin)>SERVERCHECKTIME){
      CheckReady();
      lastCheckin = millis();
    }
    if (!digitalRead(REED_SWITCH) && !Triggered){
      Triggered = true;
      tTime = millis();
    }else if(digitalRead(REED_SWITCH)) {
      Triggered = false;
      tTime = 0;
    }
    if (Triggered && ((millis() - tTime)>=HOME_SWITCH_ACTIVATION_TIME)){
      ledRedOn();
      beginSession();
      ledRedOff();
    }
    delay(100);
  }//End of While(1)
}//End of Loop()


void IRAM_ATTR beginSession(){
  unsigned long loopStartTime = 0;
  noDispChange=false;
  newSessionConnect();
  if (mySession==0){
    ledYellowOn();
    delay(500);
    active = false;
    return;
  } 
  active = true;//state variable that controls sample/report tasks below
  sendCalData();//This could change if for example, maze is moved after powerup
  
  uint32_t tTime = 0;
  TaskHandle_t * SampleA_Task_Handle;
  TaskHandle_t * SampleH_Task_Handle;
  TaskHandle_t * ReportH_Task_Handle;
  TaskHandle_t * spin_Task_Handle;
  //TaskHandle_t * serverCheck_Task_Handle;

  if (serverOK){
    //Data Collect, process and Send Tasks....
    //xTaskCreatePinnedToCore(function,pcname,stackdepth,pvparameters,priority...)
    xTaskCreate(sampleAccelTask,"SampAccelTask",4000,NULL,21,SampleA_Task_Handle);
    xTaskCreate(sampleHallTask,"SampHallTask",4000,NULL,21,SampleH_Task_Handle);
    xTaskCreate(reportHallTask,"ReportHallTask",4000,NULL,20,ReportH_Task_Handle);
    xTaskCreate(spinnerTask,"SpinnerTask",1500,NULL,13,spin_Task_Handle);
    //xTaskCreate(serverCheckInTask,"ServerCheckTask",10000,NULL,14,serverCheck_Task_Handle);
    while(active && WiFi.status()==WL_CONNECTED){//The main data collection loop
      loopStartTime=millis();
      delay(100);
      if ((loopStartTime - tTime)>=1000){
        if (digitalRead(REED_SWITCH)){
          active = false;
          tTime = loopStartTime;
          xSemaphoreTake(UDPMutexA, portMAX_DELAY);
          xSemaphoreTake(UDPMutexH, portMAX_DELAY);
          endSession();
        }else (tTime = loopStartTime);
      }
    }//End of While(Active)
    ledRedOff();
  }
  ledRedOff();
}

void endSession(){
  for (int i = 0; i<MSGBUFFERSIZE; i++){
    aMsgBuffer[i] = 0;
    hMsgBuffer[i] = 0;
  }
    xSemaphoreGive(UDPMutexH);
    xSemaphoreGive(UDPMutexA);
    StaticJsonDocument <75> byeDoc;
    byeDoc[KEY_ENDSESSION] = micros();
    byeDoc[KEY_SESSIONID] = mySession;
    uint8_t buffer[75];
    uint8_t msgLen = serializeMsgPack(byeDoc,buffer);
    wifiUDPclient.beginPacket(srvAddress,srvPort);
    wifiUDPclient.write(buffer,msgLen);
    wifiUDPclient.endPacket();
    lastSession = mySession;
}

/*  sendCalData
*     Sends prefilled moving average sample data for all sensors to the server
*     Data used for position tracking algorithm and server uses this packet to indicate a new session has started.
*/
void sendCalData(){
  uint32_t sTimer = millis();
  int count = 0;
  //Scan the Hall Matrix HALLAVERAGINGSAMPLES times, every HALLSAMPPERIOD milliseconds
  //Basically prefill the moving average.
  while(count<HALLAVERAGINGSAMPLES){
    if ((millis()-sTimer) >=10){//Sensors can be scanned every 4 ms, so this is fine to be 10ms
      sTimer = millis();
      scanHallMatrix();
      count++;
    }     
   }
  StaticJsonDocument <1500>calDoc;
  calDoc[KEY_SESSIONID] = mySession;
  calDoc[KEY_BIOSAVX] = BiotSavartCalConstantX;
  calDoc[KEY_BIOSAVY] = BiotSavartCalConstantY;
  if (sendHZ) calDoc[KEY_BIOSAVZ] = BiotSavartCalConstantZ;

  JsonArray calX = calDoc.createNestedArray(KEY_HCALX);
  JsonArray calY = calDoc.createNestedArray(KEY_HCALY);
  JsonArray calZ = calDoc.createNestedArray(KEY_HCALZ);

  for (int index = 0; index<NUMSENSORS;index++){
      calX.add(sensors[index].avgX);
      calY.add(sensors[index].avgY);
      if (sendHZ)calZ.add(sensors[index].avgZ);
  }
  int jsize = 0;
  if(DBGSERIAL){
    jsize = calDoc.memoryUsage();
    Serial.print("calDoc Mem Usage: ");
    Serial.println(jsize,DEC);
  }
  uint8_t msgPackBuffer[MSGBUFFERSIZE];
  uint16_t bufLen = serializeMsgPack(calDoc,msgPackBuffer);
  if (DBGSERIAL)Serial.print("calDoc msgpak size: ");
  if (DBGSERIAL)Serial.println(bufLen);
  wifiUDPclient.beginPacket(srvAddress,srvPort);
  wifiUDPclient.write(msgPackBuffer,bufLen);
  wifiUDPclient.endPacket();
  calDoc.clear();
}

void IRAM_ATTR scanHallMatrix(){
  for(int sIndex = 0; sIndex <NUMSENSORS; sIndex++){
    readALS31300ADC(sIndex);
  }
}

// readALS31300ADC
// Assumes hall sensor is in full loop mode.
void IRAM_ATTR readALS31300ADC(uint8_t index){
  const uint32_t requestSize = 8;
  MASensor* currSensor = &sensors[index];  //This curses us with -> below.  TODO FIX THIS UP
  
  //Subtract fromt the total, the last value saved at this location in our ring buffer
  currSensor->totX = currSensor->totX - currSensor->sampX[currSensor->sampIndex];
  currSensor->totY = currSensor->totY - currSensor->sampY[currSensor->sampIndex];
  if (sendHZ)currSensor->totZ = currSensor->totZ - currSensor->sampZ[currSensor->sampIndex];
        
  // Start the read and request 8 bytes
  // which are the contents of register 0x28 and 0x29
  I2C_HS.requestFrom(currSensor->address, requestSize);
  // Read the first 4 bytes which are the contents of register 0x28
  uint32_t value0x28 = I2C_HS.read() << 24;
  value0x28 += I2C_HS.read() << 16;
  value0x28 += I2C_HS.read() << 8;
  value0x28 += I2C_HS.read();
  // Read the next 4 bytes which are the contents of register 0x29
  uint32_t value0x29 = I2C_HS.read() << 24;
  value0x29 += I2C_HS.read() << 16;
  value0x29 += I2C_HS.read() << 8;
  value0x29 += I2C_HS.read();
  
  // Take the most significant byte of each axis from register 0x28 and combine it with the least
  // significant 4 bits of each axis from register 0x29, then sign extend the 12th bit.
  currSensor->sampX[currSensor->sampIndex] = SignExtendBitfield(((value0x28 >> 20) & 0x0FF0) | ((value0x29 >> 16) & 0x0F), 12);
  currSensor->sampY[currSensor->sampIndex] = SignExtendBitfield(((value0x28 >> 12) & 0x0FF0) | ((value0x29 >> 12) & 0x0F), 12);
  if (sendHZ)currSensor->sampZ[currSensor->sampIndex] = SignExtendBitfield(((value0x28 >> 4) & 0x0FF0) | ((value0x29 >> 8) & 0x0F), 12);

  //Update moving average total
  currSensor->totX= currSensor->totX + currSensor->sampX[currSensor->sampIndex];
  currSensor->totY= currSensor->totY + currSensor->sampY[currSensor->sampIndex];
  if (sendHZ)currSensor->totZ= currSensor->totZ + currSensor->sampZ[currSensor->sampIndex];
  
  currSensor->sampIndex = currSensor->sampIndex +1;
               
  //Ring buffer index wrap-around
  currSensor->sampIndex=currSensor->sampIndex%HALLAVERAGINGSAMPLES;

  //Update average value for sensor (TRUNCATED TO INTEGERS....)
  currSensor->avgX= (currSensor->totX) / (int16_t)HALLAVERAGINGSAMPLES;
  currSensor->avgY= (currSensor->totY) / (int16_t)HALLAVERAGINGSAMPLES;
  if (sendHZ)currSensor->avgZ= (currSensor->totZ) / (int16_t)HALLAVERAGINGSAMPLES;
  //Truncated to Int reduces data to transmit by more than 50%
}

/* read
// Using I2C, read 32 bits of data from the address on the device at the bus address
*/
uint16_t read(int busAddress, uint8_t address, uint32_t& value)
{
    // Write the address that is to be read to the device
    I2C_HS.beginTransmission(busAddress);
    I2C_HS.write(address);
    int error = I2C_HS.endTransmission(false);

    // if the device accepted the address,
    // request 4 bytes from the device
    // and then read them, MSB first
    if (error == kNOERROR)
    {
        I2C_HS.requestFrom(busAddress, 4);
        value = I2C_HS.read() << 24;
        value += I2C_HS.read() << 16;
        value += I2C_HS.read() << 8;
        value += I2C_HS.read();
    }
    return error;
}

/*
  write
    Using I2C, write 32 bit data to an address to the device at the bus address
*/
uint16_t write(int busAddress, uint8_t address, uint32_t value)
{
    // Write the address that is to be written to the device
    // and then the 4 bytes of data, MSB first
    I2C_HS.beginTransmission(busAddress);
    I2C_HS.write(address);
    I2C_HS.write((byte)(value >> 24));
    I2C_HS.write((byte)(value >> 16));
    I2C_HS.write((byte)(value >> 8));
    I2C_HS.write((byte)(value));    
    return I2C_HS.endTransmission();
}

/*
  SignExtendBitfield
      Sign extend a right justified value
*/
long IRAM_ATTR SignExtendBitfield(uint32_t data, int width)
{
    long x = (long)data;
    long mask = 1L << (width - 1);
    if (width < 32) x = x & ((1 << width) - 1); // make sure the upper bits are zero
    return (long)((x ^ mask) - mask);
}

//Enter CUSTOMER ACCESS MODE on hall sensor, allows you to write to registers
//to configure hall sensor as well as set I2C address in EEPROM
void enterCam(uint8_t busAddress, uint32_t magicKey){
  uint16_t wireError = write(busAddress, CAM_REGISTER, magicKey);
  if (wireError != kNOERROR){
    if (DBGSERIAL){
      Serial.print("Error while trying to enter customer access mode for sensor @0x");
      Serial.print(busAddress,HEX);
      Serial.print("\terror = ");
      Serial.println(wireError);
    }
  }else{
    //No actual way to check status of CAM_REGISTER so we can only assume that if the i2c transmission was ok, it worked!
  }  
}

bool initHallSensor(uint8_t index){
  enterCam(sensorAddresses[index], CAM_KEY);
  bool sensorOk = true;
  uint8_t busAddress = sensorAddresses[index];
  uint32_t value0x27;
  // Read the register the I2C loop mode is in
  uint16_t error = read(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        //Serial.print("Unable to read the ALS31300. error = ");
        //Serial.println(error);
        sensorOk = false;
    }
    // I2C loop mode is in bits 2 and 3 so mask them out
    //We want full loop mode so 3:2 should be 10,  
    value0x27 = (value0x27 & 0xFFFFFFF3) | (0b1000 );   
    // Write the new values to the register the I2C loop mode is in
    error = write(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        //Serial.print("Unable to write to the ALS31300. error = ");
        //Serial.println(error);
        sensorOk = false;
    }
    //Now set the address of the loop mode increment.
    delay(5);
    I2C_HS.beginTransmission(sensorAddresses[index]);
    I2C_HS.write(0x28);
    I2C_HS.endTransmission();
  
  //Fill Moving Average sample windows with zeros to start
  for (int i =0; i<HALLAVERAGINGSAMPLES; i++){
    sensors[index].sampX[i]=0;
    sensors[index].sampY[i]=0;
    sensors[index].sampZ[i]=0;
  }
  //Initialize other values to 0
  sensors[index].totX = 0;
  sensors[index].totY = 0;
  sensors[index].totZ = 0;
  sensors[index].avgX = 0;
  sensors[index].avgY = 0;
  sensors[index].avgZ = 0;
  sensors[index].sampIndex=0;
  
  sensors[index].address=sensorAddresses[index];
  return sensorOk;
}

void initI2CBusses(){
  I2C_HS.begin(SDA_HS, SCL_HS);
  I2C_HS.setClock(1000000);//TODO - Verify Clock Rate with Scope/LA
  //MS bus, used for accelerometer
  I2C_MS.begin(SDA_MS, SCL_MS);
  I2C_MS.setClock(400000);//Pretty close to 400KHz as verified by LA1034
}

void IRAM_ATTR pktzAndSendAccelTask( void *pvParameters){
    xSemaphoreTake(aDocMutex, portMAX_DELAY);
    
    if (DBGSERIAL){
      Serial.print("AccelDoc Mem Usage: ");
      int jsize = accelDoc.memoryUsage();
      Serial.println(jsize,DEC);
    }
    int mSize = serializeMsgPack(accelDoc,aMsgBuffer);
    xSemaphoreGive(aDocMutex);
    aSendTaskUDP.beginPacket(srvAddress,srvPort);
    
    aSendTaskUDP.write(aMsgBuffer,mSize);
    aSendTaskUDP.endPacket();
    if (DBGSERIAL){
      Serial.print("AccelDoc msgpack Size: ");
      Serial.println(mSize,DEC);
      Serial.print("A Send TASK HW MARK: ");
      Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
    }
    
    vTaskDelete(NULL);
}

void IRAM_ATTR sampleAccelTask( void * pvParameters){
  int accelTimeStamp = 0;
  TickType_t xLastRunTime;
  uint8_t accelPacketIndex = 0;//reset the sample index within the packet.
  const TickType_t xTaskDelayTimeA = ACCELSAMPPERIOD;

  //Setup for first packet
  accelDoc[KEY_ACCELTIME] =accelTimeStamp;
  accelDoc[KEY_SESSIONID] = mySession;
  Ax = accelDoc.createNestedArray("x");
  Ay = accelDoc.createNestedArray("y");
  Az = accelDoc.createNestedArray("z");
  xSemaphoreTake(UDPMutexA,portMAX_DELAY);
  bool dataReady = false;

  while(active){
    xLastRunTime = xTaskGetTickCount();
    if (dataReady){
      xTaskCreate(pktzAndSendAccelTask,"PackNSendA",2000,NULL,22,NULL);
      delayMicroseconds(95);
      xSemaphoreTake(aDocMutex, portMAX_DELAY);
      accelPacketIndex = 0;//reset the sample index within the packet.
      accelDoc.clear();//Clear out old information in the Json Structure.
      accelTimeStamp +=(accelBlockSize*ACCELSAMPPERIOD);//Increment the timestamp by the block size
      accelDoc[KEY_ACCELTIME] =  accelTimeStamp;//Prepend the timestamp of the first next sample to the JSON document
      accelDoc[KEY_SESSIONID] = mySession;
      //Recreate the nested arrays for next time
      Ax = accelDoc.createNestedArray("x");
      Ay = accelDoc.createNestedArray("y");
      Az = accelDoc.createNestedArray("z");
      dataReady=false;
      xSemaphoreGive(aDocMutex);

    }else{
      if (accelPacketIndex==0  && SENDSTS){
        accelDoc[KEY_REALSTART]= millis();
      }
      accelDoc["x"].add(accel.getRawX());
      accelDoc["y"].add(accel.getRawY());
      accelDoc["z"].add(accel.getRawZ());
      accelPacketIndex++;
      dataReady=(accelPacketIndex%accelBlockSize==0);
    }
    if (!dataReady){
      //vTaskDelay(ACCELSAMPPERIOD);
      xTaskDelayUntil(&xLastRunTime, xTaskDelayTimeA);
      //Deadlines are missed but I'm not sure we care in the end
    }else if (SENDSTS) accelDoc[KEY_REALSTOP]= millis();
  }
  if (DBGSERIAL) Serial.print("ACCEL TASK HW MARK: ");
  if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
  accelDoc.clear();
  xSemaphoreGive(UDPMutexA);
  vTaskDelete(NULL);
}

void IRAM_ATTR sampleHallTask(void * pvParameters){
  TickType_t lastSampleTime;
  BaseType_t deadlineMissed;
  const TickType_t taskDelayTime = HALLSAMPPERIOD;
  while(active){
    lastSampleTime = xTaskGetTickCount();
    xSemaphoreTake(hSampMutex, portMAX_DELAY);
    scanHallMatrix();
    xSemaphoreGive(hSampMutex);
    if (active) deadlineMissed = xTaskDelayUntil(&lastSampleTime, taskDelayTime);
  }
  if (DBGSERIAL) Serial.print("HALL Sample TASK HW MARK: ");
  if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
  vTaskDelete(NULL);
}

void IRAM_ATTR pktzAndSendHallTask( void *pvParameters){
    xSemaphoreTake(UDPMutexH,portMAX_DELAY);
    xSemaphoreTake(hDocMutex, portMAX_DELAY);
    if (DBGSERIAL){
      int jsize = hallDoc.memoryUsage();
      Serial.print("HallDoc Mem Usage: ");
      Serial.println(jsize,DEC);
    }
    int mSize = serializeMsgPack(hallDoc,hMsgBuffer);
    xSemaphoreGive(hDocMutex);
    hSendTaskUDP.beginPacket(srvAddress,srvPort);
    if (DBGSERIAL){
      Serial.print("HallDoc msgpack Size: ");
      Serial.println(mSize,DEC);
    }
    hSendTaskUDP.write(hMsgBuffer,mSize);
    hSendTaskUDP.endPacket();
    xSemaphoreGive(UDPMutexH);
    if (DBGSERIAL) Serial.print("HSend TASK HW MARK: ");
    if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
    vTaskDelete(NULL);
}

void IRAM_ATTR reportHallTask( void * pvParameters){
  int hallTimeStamp = 0;
  TickType_t lastReportTime;
  const TickType_t taskDelayTime = HALLREPORTTIME;
  BaseType_t xDeadlineMissed;
  hallDoc[KEY_HALLTIME] = hallTimeStamp;
  hallDoc[KEY_SESSIONID] = mySession;
  //int thing = CONFIG_ESP32_WIFI_TX_BUFFER_TYPE;//Convenient link to sdkconfig.h
  uint8_t hallPacketIndex = 0;
  bool dataReady=false;
  HallData = hallDoc.createNestedArray(KEY_HALLDATA);

  while(active){
    lastReportTime = xTaskGetTickCount();
    if (dataReady){ //Buffer full, time to send
      xTaskCreate(pktzAndSendHallTask,"PackNSendH",2000,NULL,22,NULL);
        
      hallPacketIndex = 0; //Reset the packet index for our next data block
      hallTimeStamp +=(hallBlockSize*HALLREPORTTIME); //Increment the timestamp by the blocksize
      dataReady = false;
      
      delayMicroseconds(95); //Wait for previous task to get established
      xSemaphoreTake(hDocMutex, portMAX_DELAY);
      hallDoc.clear();
      hallDoc[KEY_HALLTIME] =  hallTimeStamp; //Set the starting timestamp of the next packet
      hallDoc[KEY_SESSIONID] = mySession;
      HallData = hallDoc.createNestedArray(KEY_HALLDATA);
      xSemaphoreGive(hDocMutex);
    
      }else{// add current readings to hallDoc
        if (hallPacketIndex==0 && SENDSTS) hallDoc[KEY_REALSTART]= millis();
        
        //Populate the nested arrays with the actual sensor values
        xSemaphoreTake(hSampMutex, portMAX_DELAY);
        for(int sIndex = 0; sIndex <(NUMSENSORS); sIndex++){
          HallData.add(sensors[sIndex].avgX );
          HallData.add(sensors[sIndex].avgY );
          if (sendHZ) HallData.add(sensors[sIndex].avgZ );
        }
        xSemaphoreGive(hSampMutex);
        hallPacketIndex++;
        dataReady = (hallPacketIndex%hallBlockSize == 0); 
      }
      if (!dataReady){
        xDeadlineMissed = xTaskDelayUntil(&lastReportTime, taskDelayTime);
        if (SENDSTS) hallDoc[KEY_REALSTOP] = millis();
      }
    }//End While(active)
  if (DBGSERIAL) Serial.print("HALL Report TASK HW MARK: ");
  if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
  hallDoc.clear();
  vTaskDelete(NULL);
}

void initIO(){
  pinMode(_CTL_VCC3V3, OUTPUT);
  pinMode(_CTL_ACCEL_PWR, OUTPUT);
  pinMode(CTL_VDIVIDER, OUTPUT);
  pinMode(_CTL_CHA1_PWR, OUTPUT);
  pinMode(_CTL_CHA2_PWR, OUTPUT);
  pinMode(_CTL_CHA3_PWR, OUTPUT);
  pinMode(_CTL_CHA4_PWR, OUTPUT);
  pinMode(IND_LED1,OUTPUT);  //Green LED
  pinMode(IND_LED2, OUTPUT); //Red LED
  pinMode(IND_LED3, OUTPUT); //Yellow LED
  pinMode(REED_SWITCH, INPUT);
}
void initPower(){
  powerOffSensors();
  powerOnSensors(true);
}

void powerOffSensors(){
  delay(50); //Data sheet indicates 50ms required for EEPROM WRITE to finish
  digitalWrite(CTL_VDIVIDER, HIGH); //If this is left floating, will attempt to keep chips powered....bad!
  digitalWrite(_CTL_CHA1_PWR, HIGH);
  delay(100); //TUNING - Scope to find actual fall time of power rail.....
}
void powerOnSensors(bool useEEPROM){
  delay(10);
  digitalWrite(_CTL_ACCEL_PWR, LOW);//Accel On
  delay(10);
  digitalWrite(_CTL_VCC3V3,LOW); //3v3 on
  delay(50);
  if (useEEPROM){//Use addresses stored in EEPROM
    //Switch N-Fet out of circuit and allow divider taps to float up to VCC
    digitalWrite(CTL_VDIVIDER, LOW);
  }else{//Use Addresses assigned via V-Divider
    //Enable voltage divider for address pins
    digitalWrite(CTL_VDIVIDER, HIGH);
  }
  delay(10);//TUNING - Scope to find actual transition Time.
  digitalWrite(_CTL_CHA1_PWR, LOW);//Turn on Power to CHA1
  digitalWrite(_CTL_CHA2_PWR, LOW);//Turn on Power to CHA2
  delay(200); //TUNING - Scope to find actual rise time and stabilization period from datasheet.
}

void newSessionConnect(){
  //Send one hello, and wait for a response for a while....two hellos can mean a double session start
  enum ServerResponseStates serverResponseResult;
  oled.set1X();
  oled.clear();
  oled.println("Contacting Server...");
  serverResponseResult = RESP_TIMEOUT;
  StaticJsonDocument<500> startDoc;
  startDoc[KEY_FWVER] = VERSIONSTRING;
  startDoc[KEY_STARTSESSION] = micros();
  startDoc[KEY_NUMHALLSENSORS] = NUMSENSORS;
  startDoc[KEY_HALLBLOCKSIZE] = hallBlockSize;
  startDoc[KEY_ACCELBLOCKSIZE] = accelBlockSize;
  startDoc[KEY_HALLHASZ] = 1;
  uint8_t msgPackBuffer[500];
  uint16_t bufLen = serializeMsgPack(startDoc,msgPackBuffer);
  wifiUDPclient.beginPacket(srvAddress,srvPort);
  wifiUDPclient.write(msgPackBuffer,bufLen);
  wifiUDPclient.endPacket();
  startDoc.clear();
  int waitLoops = 0;
  while((serverResponseResult!=RESP_OK) && waitLoops<20){
    delay(100);
    uint8_t buffer[250];
    StaticJsonDocument<300> serverResponse;
    int pSize = wifiUDPclient.parsePacket();
    if (pSize){
      int len = wifiUDPclient.read(buffer,250);
      DeserializationError de_error = deserializeMsgPack(serverResponse, buffer);
      if (de_error){
        serverResponseResult = RESP_BAD_DE;
      }else{
        bool p1 = serverResponse.containsKey(KEY_SESSIONID);
        bool p2 = serverResponse.containsKey(KEY_SENDHZ);
        bool p3 = serverResponse.containsKey(KEY_ACCELSAMPLETIME);
        bool p4 = serverResponse.containsKey(KEY_HALLSAMPLETIME);
        bool p5 = serverResponse.containsKey(KEY_HALLMASIZE);
        bool p6 = serverResponse.containsKey(KEY_HALLREPORTTIME);
        bool p7 = serverResponse.containsKey(KEY_SENDSTS);
        bool AllKeysPresent = p1&&p2&&p3&&p4&&p5&&p6&&p7;
        if (AllKeysPresent){
          mySession = serverResponse[KEY_SESSIONID];
          if (mySession>0){
            serverResponseResult = RESP_OK;
            sendHZ = serverResponse[KEY_SENDHZ];
            ACCELSAMPPERIOD = serverResponse[KEY_ACCELSAMPLETIME];
            HALLSAMPPERIOD = serverResponse[KEY_HALLSAMPLETIME];
            HALLAVERAGINGSAMPLES = serverResponse[KEY_HALLMASIZE];
            HALLREPORTTIME = serverResponse[KEY_HALLREPORTTIME];
            SENDSTS = serverResponse[KEY_SENDSTS];
          }
        }else{
          serverResponseResult = RESP_BAD_MK;//Missing Keys
        }
      }
    }else{
      waitLoops++;
    }
  }
  if (serverResponseResult==RESP_OK){
    oled.setCursor(0,0);
    oled.clearToEOL();
    oled.set2X();
    oled.println("  Session\n   Active");
    oled.set2X();
    oled.setCursor(32,6);
    oled.print("#:");
    oled.print(mySession,DEC);
    oled.set1X();
    ledRedOn();
  }else if (serverResponseResult == RESP_TIMEOUT){
    oled.clear();
    oled.set2X();
    oled.println("  Server");
    oled.println("  Timeout");
    
  }else if (serverResponseResult == RESP_BAD_DE){
    oled.clear();
    oled.set2X();
    oled.println("Bad Reply\n[Dec. Error]");
  }else if (serverResponseResult == RESP_BAD_MK){
    oled.clear();
    oled.set2X();
    oled.println("Bad Reply\n[Mis. Key]");
  }
  if (serverResponseResult!=RESP_OK){
    active=false;
    oled.set1X();
    mySession = 0;
    ledYellowOff();
    oled.setCursor(0,6);
    oled.clearToEOL();
    oled.println("     Home to Continue");
    noDispChange=false;  
    waitStylusHome(true);
  }
  
}

bool findServer(bool updateDisplay){
  bool serverFound = false;
  if (updateDisplay){
    oled.set1X();
    oled.clear();
    oled.print("Listen for Server...");
    oled.setCursor(0,1);
  }
  wifiUDPclient.begin(bcastPort);
  
  bool serviceMatch = false;
  bool versionMatch = false;

  while(!(serviceMatch && versionMatch)){
    uint8_t buffer[400];
    StaticJsonDocument<399> findServerDoc;
    int pSize = wifiUDPclient.parsePacket();
    if (pSize){
      int len = wifiUDPclient.read(buffer,399);
      DeserializationError de_error = deserializeMsgPack(findServerDoc, buffer);
      if (de_error){
        if (updateDisplay){
          oled.setCursor(0,7);
          oled.print("Bad BCAST Packet");
        }
      }else{
        if (updateDisplay){
          oled.setCursor(0,4);
          oled.clearToEOL();
        }
        String serviceTag = findServerDoc[KEY_SERVICETYPE];
        serviceMatch = (serviceTag==SERVICEREQUIRED);
        if (serviceMatch){
          float verTag = findServerDoc[KEY_SERVICEVER];
          versionMatch = (verTag>=MINSERVERVER);
          srvPort = findServerDoc[KEY_SERVICEPORT];
          srvAddress=wifiUDPclient.remoteIP();
          serverFound = true;
          if (updateDisplay){
            oled.print(serviceTag);
            oled.print(":");
            oled.setCursor(0,6);
            oled.clearToEOL();
            oled.print("@");
            oled.print(wifiUDPclient.remoteIP());
            oled.setCursor(0,7);
            oled.clearToEOL();
            oled.print("V:");
            oled.print(verTag,2);
            oled.setCursor(51,7);
            oled.print("P:");
            oled.print(srvPort,DEC);
            if (srvPort == SERVICEBROADCASTPORT ) oled.print("(!)");
            oled.setCursor(0,1);
            oled.clearToEOL();
            oled.print("Server Found!");
          }
        }
      }
    }
  }
  return serverFound;
}

void hallSensorZeroCal(){
  oled.clear();
  oled.set2X();
  oled.print("Waiting For\nStylusHome");

  oled.set1X();
  waitStylusHome(true);
  uint32_t sTime = millis();
  uint32_t cTime=0;
  delay(HOME_SWITCH_ACTIVATION_TIME);
  int count = 0;
  while(count<HALLAVERAGINGSAMPLES){
    cTime = millis();  
    if((cTime - sTime)>=5){//We can read this faster than 4ms since it's only one sensor.
      sTime = cTime;
      readALS31300ADC(calSensorIndexLocation);
      count++;
    }
  }
  BiotSavartCalConstantX = sensors[calSensorIndexLocation].avgX;
  BiotSavartCalConstantY = sensors[calSensorIndexLocation].avgY;
  BiotSavartCalConstantZ = sensors[calSensorIndexLocation].avgZ;
  oled.clear();
  oled.set2X();
  oled.println("----------");
  oled.println("HallSensor");
  oled.println("Calibrated");
  oled.println("----------");
}

void waitStylusHome(bool animate){
  uint32_t sTime = millis();
  uint32_t cTime=0;
  uint8_t dotCount = 0;
  bool fwd = true;
  while(!digitalRead(REED_SWITCH)){ //Wait for reed switch to be in place before gathering data
    if (animate){
      oled.setCursor(dotCount*6,7);
      if (millis()-sTime >= 50){
        if (fwd){
          oled.print(" ");
          oled.print("?");
          dotCount++;
        }else{
        oled.print("?");
        oled.print(" ");
        dotCount--;
        }
        sTime = millis();
      }
      if (dotCount==20){
        fwd=false;
      }
      if (dotCount==0){
        fwd=true;
      }
    }
    delay(60);
  }
  if (animate){
    oled.setCursor(0,7);
    oled.clearToEOL();
    noDispChange = false;
  }
  if (animate) noDispChange = false;
}

void spinnerTask( void * pvParameters){
  bool recLed = true;
  uint32_t blinkTime = 0;
  while(active){
    if ((millis() - blinkTime)>=1000){
      blinkTime = millis();
      recLed = !recLed;
      if (recLed) ledRedOn();
      if (!recLed) ledRedOff();
    }
    oled.setCursor(112,7);
    switch (SpinState){
      case spin_H:
        oled.print("(-)");
        SpinState=spin_LS;
      break;
      case spin_LS:
        oled.print("(\\)");
        SpinState=spin_V1;
      break;
      case spin_V1:
        oled.print("(|)");
        SpinState=spin_RS;
      break;
      case spin_RS:
        oled.print("(/)");
        SpinState=spin_H;
      break;
      default:
      break;
      
    }
    delay(SPINNERTASKDELAY);
  }
  ledRedOff();
  noDispChange = false;
  vTaskDelete(NULL);
}

/*
void serverCheckInTask(void * pvParameters){
  while(active){
    CheckServerAlive(false,false);
    if (!serverOK){
      active = false;
      delay(10);
      oled.clear();
      oled.set2X();
      oled.println("Server Lost\n During\n Session");
      oled.set1X();
    }
    delay(SERVERCHECKTIME);
  }
}*/