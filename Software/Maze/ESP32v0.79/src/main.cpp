/*
 *  Example code for reading an ALS31313 3D Hall Effect Sensor on ESP32 in Arduino environment.
 *  VintLabs 3dHall Module available at https://www.amazon.ca/dp/B0896VD73G
 *  
 *  Compile and run - I recommend using the serial plotter rather than the serial console for testing!
 *  
 *  
 *  Much of the code is taken from the example code provided by Allegro Microsystems:
 *    Example source code for an Arduino to show
 *    how to communicate with an Allegro ALS31300
 *
 *    Written by K. Robert Bate, Allegro MicroSystems, LLC.
 *
 *    ALS31300Demo is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
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

#define VERSIONSTRING "v0.79" //Firmware Version String
//WiFi credentials
#define G15_SSID "AMazeThing"
#define G15_PASSWORD "hn7P8Egh"

#define MSGBUFFERSIZE 1500 //Size of char buffers to serialze msgpack data into (2x buffers)

#define SERVERCHECKTIME 3000 //Check server alive every nnnn/1000 seconds
#define HOME_SWITCH_ACTIVATION_TIME 300 //Time to wait after inital switch open before starting a new session (Part of debounce)

#define DBGSERIAL 1 //Output various information over serial  (Used during parameter tuning)

//Hall Sensor Matrix Properties
#define HALLAVERAGINGSAMPLES 20 //Number of samples in moving average
#define NUMSENSORS 24 //Should be 24 in final project
#define HALLSAMPPERIOD 25 //4.5ms is absolute min delay between samples
#define HALLREPORTTIME 200 //Report time interval *THIS SHOULD BE A MULTIPLE OF HALLSAMPPERIOD*
#define BLOCKSIZEHALL  13 //14 is about the maximum we can go before fragmentation will start to become an issue - MSGBUFFERSIZE (based on MTU) is the limitation else we start spanning packets

//Accelerometer Properties
#define ACCELSAMPPERIOD 40// 1/(40/1000) = 25 Hz Sample Rate --> Nyquist 20Hz
#define BLOCKSIZEACCEL 100

//ALS31313 Hall Sensor parameters (from datasheet)
#define EEPROMDEFAULT     0b00000000000000000000001111100000
#define EEPROMADDRESSMASK 0b11111111111111100000001111111111
#define CAM_REGISTER 0x35
//#define CAM_KEY 0x2C41354 //Was incorrect in example code //ALS31313 Sensors
#define CAM_KEY 0x2C413534 //ALS31300 Sensors


//Server Properties to look for (and where to look)
#define MINSERVERVER 0.79
#define SERVICEREQUIRED "MAZE_SERVER"
#define SERVICEBROADCASTPORT 7777
//MSGPACK (JSONDOC) KEYS
#define KEY_HALLTIME "TH"
#define KEY_ACCELTIME "TA"
#define KEY_ENDSESSION "Bye"
#define KEY_STARTSESSION "Hello"
#define KEY_BIOSAVX "BSX"
#define KEY_BIOSAVY "BSY"
#define KEY_BIOSAVZ "BSZ"
#define KEY_HCALX "H_CAL_X"
#define KEY_HCALY "H_CAL_Y"
#define KEY_HCALZ "H_CAL_Z"
#define KEY_SRVCHECK "CHK"
#define KEY_SRVOK "OK"
#define KEY_SESSIONID "ID"
#define KEY_REALSTART "S"
#define KEY_REALSTOP "E"
#define KEY_HALLSAMPLETIME "HST"
#define KEY_NUMHALLSENSORS "NumSens"
#define KEY_HALLBLOCKSIZE "HBS"
#define KEY_ACCELSAMPLETIME "AST"
#define KEY_ACCELBLOCKSIZE "ABS"
#define KEY_HALLHASZ "HallHasZ"
#define KEY_SENDHZ "WantHZ"

#define KEY_SERVICETYPE "SVC"
#define KEY_SERVICEVER "VER"
#define KEY_SERVICEPORT "PORT"


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

//Moving Average Struct to store Hall sensor readings
typedef struct{
  uint8_t sampIndex;
  int16_t sampX[HALLAVERAGINGSAMPLES];
  int16_t sampY[HALLAVERAGINGSAMPLES];
  int16_t sampZ[HALLAVERAGINGSAMPLES];
  int16_t avgX,avgY,avgZ;
  int totX,totY,totZ;
  uint8_t address; 
} MASensor;
MASensor sensors[NUMSENSORS];

enum SpinTaskStates{
  spin_H,
  spin_LS=1,
  spin_V=2,
  spin_RS=3
};
enum SpinTaskStates SpinState;

bool sendHZ = true;

float BiotSavartCalConstantX = 0.0;
float BiotSavartCalConstantY = 0.0;
float BiotSavartCalConstantZ = 0.0;

//Each packet has one T = Key, increments by whatever hallBlockSize*sampTime(50mS) or accelBlockSize*sampTime(10ms) is
int hallTimeStamp = 0;
int accelTimeStamp = 0;

//Keeps track of current time stamp within a bundled packet (increments and resets at hallBlockSize/accelBlockSize)
uint8_t hallPacketIndex = 0;
uint8_t accelPacketIndex = 0;

//Number of matrix scans/Accelerometer reads to bundle into a packet
const uint8_t hallBlockSize = BLOCKSIZEHALL;
const uint8_t accelBlockSize = BLOCKSIZEACCEL;

//Json stores hallBlockSize sample periods worth of data.
StaticJsonDocument <(1250 * hallBlockSize)> hallDoc;//TUNING - Recalculate packet size based on new Hx# values
uint8_t hMsgBuffer[MSGBUFFERSIZE];//msgPack Buffer for Hall
JsonArray Hx[hallBlockSize];
JsonArray Hy[hallBlockSize];
JsonArray Hz[hallBlockSize];

JsonArray Ax,Ay,Az;

//Used for listening to service announce, checking, initiating and ending sessions with server
WiFiUDP wifiUDPclient;

uint16_t srvPort = 20002; //Default port is 20002, but this is actually set by service broadcast at startup
const uint16_t bcastPort = SERVICEBROADCASTPORT; //Port that device listens to service broadcasts on.
IPAddress srvAddress;


//We generate these once below instead of sprintf'ing our way to new strings each packet
//This saves a lot of time
char HxLabel[hallBlockSize][4];//Makes x0,x1,x2.....x'hallBlockSize'
char HyLabel[hallBlockSize][4];
char HzLabel[hallBlockSize][4];

//Json stores accelBlockSize sample periods worth of data
StaticJsonDocument <(55*accelBlockSize)>accelDoc;//TUNING - Recalculate packet size based on new Ax# values
uint8_t aMsgBuffer[MSGBUFFERSIZE]; //msgPack Buffer for Accel
//Tracking timestamps of last sample, and last report.  TODO - CONSIDER Move to Timer interrupt based sampling
unsigned long loopStartTime;//, lastHallSampleTime, lastHallReportTime, lastAccelSampleTime, lastAccelReportTime;

bool active = false;

SemaphoreHandle_t hMutex, aMutex;
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
void waitStylusHome();
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
void IRAM_ATTR packetizeAndSendH(void * pvParameters);
void IRAM_ATTR packetizeAndSendA(void * pvParameters);
void IRAM_ATTR sampleAccelTask( void * pvParameters);
void IRAM_ATTR sampleHallTask( void * pvParameters);
void newSessionConnect();
bool findServer(bool updateDisplay);
void CheckReady();
void CheckServerAlive(bool useDisp,bool findServerIfLost);

void spinnerTask( void * pvParameters);
//void serverCheckInTask(void * pvParameters);

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

  hMutex = xSemaphoreCreateMutex();//Used to prevent hall data from being reset before serialized
  aMutex = xSemaphoreCreateMutex();//Used to prevent accel data from being reset before serialized
  UDPMutexA = xSemaphoreCreateMutex();//Used to ensure sample/send tasks are stopped before closing session
  UDPMutexH = xSemaphoreCreateMutex();//Used to ensure sample/send tasks are stopped before closing session

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

  if (DBGSERIAL)Serial.begin(230400);  

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
  
  //Generate our Hall Sensor Keys in advance, avoids sprintf every packet (speedup!)
  for (int index = 0; index<hallBlockSize;index++){
    sprintf(HxLabel[index], "x%u", index);
    sprintf(HyLabel[index], "y%u", index);
    sprintf(HzLabel[index], "z%u", index);
  }
  //Serial.println("AMazeThing...\nReady state starts in 1 Sec");
  
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
  noDispChange=false;
  newSessionConnect();
  if (mySession<=0){
    oled.clear();
    oled.set2X();
    oled.println("  Server");
    oled.println(" Session #");
    oled.println("  Invalid");
    delay(1500);
    ledYellowOn();
    return;
  } 

  hallTimeStamp = 0;
  accelTimeStamp = 0;
  
  //RESET packet indexes
  hallPacketIndex = 0;
  accelPacketIndex = 0;
 
  sendCalData();//This could change if for example, maze is moved after powerup
  //Populate the initial time stamps, avoids repeated if logic in buildSendPacket
  hallDoc[KEY_HALLTIME] = hallTimeStamp;
  hallDoc[KEY_SESSIONID] = mySession;
  accelDoc[KEY_ACCELTIME] =accelTimeStamp;
  accelDoc[KEY_SESSIONID] = mySession;
  
  //Needed to avoid blank first Accel packet
  Ax = accelDoc.createNestedArray("x");
  Ay = accelDoc.createNestedArray("y");
  Az = accelDoc.createNestedArray("z");
 
  uint32_t tTime = 0;
  uint32_t blinkTime = 0;
  active = true;//state variable toggled by reed switch
  TaskHandle_t * SampleA_Task_Handle;
  TaskHandle_t * SampleH_Task_Handle;
  TaskHandle_t * spin_Task_Handle;
  //TaskHandle_t * serverCheck_Task_Handle;


  bool recLed = true;
  if (serverOK){
    
    //Serial.println("ServerOK");
    //Data Collect, process and Send Tasks....
    //xTaskCreatePinnedToCore(function,pcname,stackdepth,pvparameters,priority...)
    xTaskCreatePinnedToCore(sampleAccelTask,"SampAccelTask",4000,NULL,20,SampleA_Task_Handle,1);
    xTaskCreatePinnedToCore(sampleHallTask,"SampleHallTask",4000,NULL,20,SampleH_Task_Handle,1);
    xTaskCreate(spinnerTask,"SpinnerTask",1500,NULL,14,spin_Task_Handle);
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
  accelDoc.clear();
  hallDoc.clear();
  for (int i = 0; i<MSGBUFFERSIZE; i++){
    aMsgBuffer[i] = 0;
    hMsgBuffer[i] = 0;
  }
    xSemaphoreGive(UDPMutexH);
    xSemaphoreGive(UDPMutexA);
    StaticJsonDocument <50> byeDoc;
    byeDoc[KEY_ENDSESSION] = mySession;
    uint8_t buffer[50];
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
  //solidRedLed();
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
  int jsize = calDoc.memoryUsage();
  if (DBGSERIAL)Serial.print("calDoc Mem Usage: ");
  if (DBGSERIAL)Serial.println(jsize,DEC);
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
  //  (4400uSec)
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

  //Truncate to Int reduces data to transmit by more than 50%

}



//
// read
//
// Using I2C, read 32 bits of data from the address on the device at the bus address
//
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

//
// write
//
// Using I2C, write 32 bit data to an address to the device at the bus address
//
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

//
// SignExtendBitfield
//
// Sign extend a right justified value
//
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
    //Serial.print("Error while trying to enter customer access mode for sensor @0x");
    //Serial.print(busAddress,HEX);
    //Serial.print("\terror = ");
    //Serial.println(wireError);
  }else{
    //Serial.print("\t@0x");
    //Serial.print(busAddress,HEX);
    //No actual way to check status of CAM_REGISTER so we can only assume that if the i2c transmission was ok, it worked!
    //Serial.println(" Entered C.A.M Successfully");
  }  
}

bool initHallSensor(uint8_t index){
  enterCam(sensorAddresses[index], CAM_KEY);
  bool sensorOk = true;
  //TODO CLEAN UP CODE AND JUST USE index below!
  uint8_t busAddress = sensorAddresses[index];
  //Disable Loop Mode //Should be disabled by default
  
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

void IRAM_ATTR packetizeAndSendH( void *pvParameters){
    xSemaphoreTake(hMutex, portMAX_DELAY);
    hSendTaskUDP.beginPacket(srvAddress,srvPort);
    int jsize = hallDoc.memoryUsage();
    if (DBGSERIAL)Serial.print("HallDoc Mem Usage: ");
    if (DBGSERIAL)Serial.println(jsize,DEC);
    int mSize = serializeMsgPack(hallDoc,hMsgBuffer);
    if (DBGSERIAL)Serial.print("HallDoc msgpack Size: ");
    if (DBGSERIAL)Serial.println(mSize,DEC);
    xSemaphoreGive(hMutex);
    
    hSendTaskUDP.write(hMsgBuffer,mSize);
    hSendTaskUDP.endPacket();
    if (DBGSERIAL) Serial.print("HSend TASK HW MARK: ");
    if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
    vTaskDelete(NULL);
}

void IRAM_ATTR packetizeAndSendA( void *pvParameters){
    xSemaphoreTake(aMutex, portMAX_DELAY);
    aSendTaskUDP.beginPacket(srvAddress,srvPort);
    int jsize = accelDoc.memoryUsage();
    if (DBGSERIAL)Serial.print("AccelDoc Mem Usage: ");
    if (DBGSERIAL)Serial.println(jsize,DEC);
    int mSize = serializeMsgPack(accelDoc,aMsgBuffer);
    if (DBGSERIAL)Serial.print("AccelDoc msgpack Size: ");
    if (DBGSERIAL)Serial.println(mSize,DEC);
    xSemaphoreGive(aMutex);
    
    aSendTaskUDP.write(aMsgBuffer,mSize);
    aSendTaskUDP.endPacket();
    if (DBGSERIAL) Serial.print("A Send TASK HW MARK: ");
    if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
    vTaskDelete(NULL);
}

void IRAM_ATTR sampleAccelTask( void * pvParameters){
  TickType_t xLastRunTime;
  const TickType_t xTaskDelayTimeA = ACCELSAMPPERIOD;
  xSemaphoreTake(UDPMutexA,portMAX_DELAY);
  bool dataReady = false;
  while(active){
    xLastRunTime = xTaskGetTickCount();
    if (dataReady){
      xTaskCreate(packetizeAndSendA,"PackNSendA",2000,NULL,22,NULL);
      delayMicroseconds(100);
      xSemaphoreTake(aMutex, portMAX_DELAY);
      
      accelPacketIndex = 0;//reset the sample index within the packet.
      accelDoc.clear();//Clear out old information in the Json Structure.
      accelTimeStamp +=(accelBlockSize*ACCELSAMPPERIOD);//Increment the timestamp by the block size
      accelDoc[KEY_ACCELTIME] =  accelTimeStamp;//Prepend the timestamp of the first next sample to the JSON document
      //Recreate the nested arrays for next time
      Ax = accelDoc.createNestedArray("x");
      Ay = accelDoc.createNestedArray("y");
      Az = accelDoc.createNestedArray("z");
      dataReady=false;
      xSemaphoreGive(aMutex);

    }else{
      if (accelPacketIndex==0){
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
    }else accelDoc[KEY_REALSTOP]= millis();
  }
  if (DBGSERIAL) Serial.print("ACCEL TASK HW MARK: ");
  if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
  xSemaphoreGive(UDPMutexA);
  vTaskDelete(NULL);
}

void IRAM_ATTR sampleHallTask( void * pvParameters){
  TickType_t xLastRunTime;
  const TickType_t xTaskDelayTimeH = HALLSAMPPERIOD;
  BaseType_t xDeadlineMissed;
  //int thing = CONFIG_ESP32_WIFI_TX_BUFFER_TYPE;//Convenient link to sdkconfig.h
  xSemaphoreTake(UDPMutexH,portMAX_DELAY);
  bool dataReady=false;
  uint16_t sampCount = 0;
  while(active){
    xLastRunTime = xTaskGetTickCount();
    scanHallMatrix();
    sampCount++;
    if (sampCount%(HALLREPORTTIME/HALLSAMPPERIOD) == 0){
      if ((hallPacketIndex % hallBlockSize)==0 && hallPacketIndex>0){
        //int st = micros(); //Takes roughly 75 uSec to create the task
        xTaskCreate(packetizeAndSendH,"PackNSendH",2000,NULL,22,NULL);
        
        delayMicroseconds(85); //Wait for previous task to get established
        xSemaphoreTake(hMutex, portMAX_DELAY);
        hallDoc.clear();
        hallPacketIndex = 0; //Reset the packet index for our next data block
        hallTimeStamp +=(hallBlockSize*HALLREPORTTIME); //Increment the timestamp by the blocksize
        hallDoc[KEY_HALLTIME] =  hallTimeStamp; //Set the starting timestamp of the next packet
        dataReady = false;
        xSemaphoreGive(hMutex);
        }else{//Add, we can add the hall sensor samples to the packet
          //int st = micros();
          if (hallPacketIndex==0){
            hallDoc[KEY_REALSTART]= millis();
          }
          //Add the nested arrays to the JSON doc for this sample
          Hx[hallPacketIndex] = hallDoc.createNestedArray(HxLabel[hallPacketIndex]);
          Hy[hallPacketIndex] = hallDoc.createNestedArray(HyLabel[hallPacketIndex]);
          if (sendHZ) Hz[hallPacketIndex] = hallDoc.createNestedArray(HzLabel[hallPacketIndex]);

          //Populate the nested arrays with the actual sensor values
          for(int sIndex = 0; sIndex <(NUMSENSORS); sIndex++){
            Hx[hallPacketIndex].add(sensors[sIndex].avgX );
            Hy[hallPacketIndex].add(sensors[sIndex].avgY );
            if (sendHZ) Hz[hallPacketIndex].add(sensors[sIndex].avgZ );
          }
          hallPacketIndex++;
          dataReady = (hallPacketIndex%hallBlockSize == 0);
          //int et = micros();
          //Serial.print("HAddXYTime (uSec): ");
          //Serial.println((et-st),DEC);

        }
    }
    if (!dataReady){
      xDeadlineMissed = xTaskDelayUntil(&xLastRunTime, xTaskDelayTimeH);
      
      //vTaskDelay(HALLSAMPPERIOD);
    }else hallDoc[KEY_REALSTOP]= millis();
  }
  if (DBGSERIAL) Serial.print("HALL SAMPLE TASK HW MARK: ");
  if (DBGSERIAL) Serial.println(uxTaskGetStackHighWaterMark(NULL),DEC);
  xSemaphoreGive(UDPMutexH);
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
  oled.set1X();
  oled.clear();
  oled.println("Contacting Server...");
  bool responseOK = false;
  StaticJsonDocument<500> startDoc;
  startDoc[KEY_STARTSESSION] = VERSIONSTRING;
  startDoc[KEY_HALLSAMPLETIME]=HALLREPORTTIME;//This is the sample rate that we sample the AVERAGE, the actual sample rate is higher
  startDoc[KEY_NUMHALLSENSORS] = NUMSENSORS;
  startDoc[KEY_HALLBLOCKSIZE] = hallBlockSize;
  startDoc[KEY_ACCELSAMPLETIME] = ACCELSAMPPERIOD;
  startDoc[KEY_ACCELBLOCKSIZE] = accelBlockSize;
  startDoc[KEY_HALLHASZ] = 1;
  uint8_t msgPackBuffer[500];
  uint16_t bufLen = serializeMsgPack(startDoc,msgPackBuffer);
  //Serial.println(bufLen);
  wifiUDPclient.beginPacket(srvAddress,srvPort);
  wifiUDPclient.write(msgPackBuffer,bufLen);
  wifiUDPclient.endPacket();
  startDoc.clear();
  int waitLoops = 0;
  while(!responseOK&& waitLoops<33){
    delay(100);
    uint8_t buffer[150];
    StaticJsonDocument<150> serverResponse;
    int pSize = wifiUDPclient.parsePacket();
    if (pSize){
      int len = wifiUDPclient.read(buffer,150);
      DeserializationError de_error = deserializeMsgPack(serverResponse, buffer);
      if (de_error){
        oled.print("Bad Greeting :(");
      }else{
        if (serverResponse.containsKey(KEY_SESSIONID)){
          mySession = serverResponse[KEY_SESSIONID];
          if (mySession>=0){
            responseOK = true;
          } 
        }
        if (serverResponse.containsKey(KEY_SENDHZ)){
          sendHZ = serverResponse[KEY_SENDHZ];
        }
      }
    }else{
      waitLoops++;
    }
  }
  if (responseOK){
    oled.setCursor(0,0);
    oled.clearToEOL();
    oled.set2X();
    oled.println("  Session\n   Active");
    oled.set2X();
    oled.setCursor(32,6);
    oled.print("#:");
    oled.print(mySession,DEC);
    oled.set1X();
  }else{
    active=false;
    oled.clear();
    oled.set2X();
    oled.println(" *Error*");
    oled.println("  Server");
    oled.println("  Timeout");
    oled.set1X();
    oled.setCursor(0,7);
    oled.print("Stylus Wait");
    mySession = -1;
    ledYellowOff();
    waitStylusHome();
  }
  ledRedOn();
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
  uint32_t sTime = millis();
  uint32_t cTime=0;
  oled.set1X();
  oled.setCursor(0,6);
  uint8_t dotCount = 0;
  sTime = millis();
  bool fwd = true;
  
  while(!digitalRead(REED_SWITCH)){ //Wait for reed switch to be in place before gathering data
    oled.setCursor(dotCount*6,6);
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
    delay(60);
  }
  delay(HOME_SWITCH_ACTIVATION_TIME);
  
  int count = 0;
  while(count<HALLAVERAGINGSAMPLES){
    cTime = millis();  
    if((cTime - sTime)>=5){//We can read this faster than 6ms since it's only one sensor.
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
void waitStylusHome(){
  while(!digitalRead(REED_SWITCH)){
    delay(50);
  }
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
        SpinState=spin_RS;
      break;
      case spin_RS:
        oled.print("(/)");
        SpinState=spin_H;
      break;
      default:
      break;
      
    }
    delay(1000);
    
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