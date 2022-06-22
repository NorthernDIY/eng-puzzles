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

#define VERSIONSTRING "v0.79"

#define G15_SSID "AMazeThing"
#define G15_PASSWORD "hn7P8Egh"

#define UDPBUFSIZE 1500


#define SERVERCHECKTIME 2500

#define HOME_SWITCH_ACTIVATION_TIME 300

//Hall Sensor Matrix Properties
#define HALLAVERAGINGSAMPLES 10 //Number of samples in moving average
#define NUMSENSORS 24 //Should be 24 in final project
#define HALLSAMPPERIOD 25 //4.5ms is absolute min delay between samples
#define HALLREPORTTIME 200 //Report time interval *THIS SHOULD BE A MULTIPLE OF HALLSAMPPERIOD*
#define BLOCKSIZEHALL  10 //10 may have fragmentation issues that cause hiccups!

//Accelerometer Properties
#define ACCELSAMPPERIOD 40// 1/(40/1000) = 25 Hz Sample Rate --> Nyquist 20Hz
#define BLOCKSIZEACCEL 40

//ALS31313 Hall Sensor parameters (from datasheet)
#define EEPROMDEFAULT     0b00000000000000000000001111100000
#define EEPROMADDRESSMASK 0b11111111111111100000001111111111
#define CAM_REGISTER 0x35
//#define CAM_KEY 0x2C41354 //Was incorrect in example code //ALS31313 Sensors
#define CAM_KEY 0x2C413534 //ALS31300 Sensors

#define MSGBUFFERSIZE 1500 //Size of char buffers to serialze msgpack data into (2x buffers)

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
#define KEY_HALLUSESZ "HallZ"

#define KEY_SERVICETYPE "SVC"
#define KEY_SERVICEVER "VER"
#define KEY_SERVICEPORT "PORT"


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


//Actual C code files C:\Users\---username---\.platformio\packages\framework-espidf\components\driver
//#include "driver/i2c.h" //Needed to allow use of I2C peripheral settings to actually reach 1MHz!
TwoWire I2C_HS = TwoWire(0);//HS I2C (Hall Sensors)
TwoWire I2C_MS = TwoWire(1);//MS I2C (Accelerometer)

SPARKFUN_LIS2DH12 accel;
#define ACCEL_ADDRESS 0x19

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

bool noDispChange = false;

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
StaticJsonDocument <(1500 * hallBlockSize)> hallDoc;//TUNING - Recalculate packet size based on new Hx# values
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
char HxLabel[hallBlockSize][5];//Makes x0,x1,x2.....x'hallBlockSize'
char HyLabel[hallBlockSize][5];
char HzLabel[hallBlockSize][5];

//Json stores accelBlockSize sample periods worth of data
StaticJsonDocument <(70*accelBlockSize)>accelDoc;//TUNING - Recalculate packet size based on new Ax# values
uint8_t aMsgBuffer[MSGBUFFERSIZE]; //msgPack Buffer for Accel
//Tracking timestamps of last sample, and last report.  TODO - CONSIDER Move to Timer interrupt based sampling
unsigned long loopStartTime;//, lastHallSampleTime, lastHallReportTime, lastAccelSampleTime, lastAccelReportTime;

bool active = false;
//This is the power up addresses of the sensors when using the voltage divider inputs to ADR0/1
//Repeated for convenience across 2 channels
uint8_t FactoryPowerUpAddresses[NUMSENSORS] = {96,97,98,99,100,101,102,103,104,105,106,107,96,97,98,99,100,101,102,103,104,105,106,107};

//Addresses of hall sensors if they have been programmed
const uint8_t sensorAddresses[] = {12,22,11,21,14,24,13,23,16,26,15,25,32,42,31,41,34,44,33,43,36,46,35,45};
const uint8_t calSensorAddress = 31;
const uint8_t calSensorIndexLocation = 14;//Location in the array above...used to reference sensor data for BiotSavart Const Calculation

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
#define ledGreenOn() digitalWrite(IND_LED1, HIGH)
#define ledGreenOff() digitalWrite(IND_LED1, LOW)
#define ledRedOn() digitalWrite(IND_LED2, HIGH)
#define ledRedOff() digitalWrite(IND_LED2, LOW)
#define ledYellowOn() digitalWrite(IND_LED3, HIGH)
#define ledYellowOff() digitalWrite(IND_LED3, LOW)

#define waitStylusHome() while(!digitalRead(REED_SWITCH))

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
void CheckServerAlive();


// 0X3C+SA0 - 0x3C or 0x3D
#define OLED_I2C_ADDRESS 0x3C

// Define proper OLED_RST_PIN if required.
#define OLED_RST_PIN -1

SSD1306AsciiWire oled(I2C_MS);
//
// setup
//
// Initializes the Wire library for I2C communications,
// Serial for displaying the results and error messages,
// the hardware and variables to blink the LED,
// and sets the ALS31300 into customer access mode.
//
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
  Serial.begin(230400);
  
  //int freq = getXtalFrequencyMhz();
  //Serial.print(F("\nSystem Frequency: "));
  //Serial.println(freq);
  //Serial.print(F("Connecting to WiFi:"));
  //Serial.print(ssid);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  oled.setCursor(0,3);
  oled.print("WiFi: ");
  while(WiFi.status() != WL_CONNECTED){
    delay(250);
    //Serial.print(".");
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

  //Development Info
  //Serial.printf("SETTINGS:,SamplePeriod:,%dms,ReportRate:,%dms,NumSensors:,%d,AvgSamples:,%d\n", HALLSAMPPERIOD,HALLREPORTTIME,NUMSENSORS,HALLAVERAGINGSAMPLES);
  
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
  CheckServerAlive();
  
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
void CheckServerAlive(){
  serverOK = false;
  oled.set1X();
  oled.setCursor(120,7);
  oled.print(".");
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
          oled.print("Bad packet Rx'd");
          noDispChange=false;
        }else if (serverResponse.containsKey(KEY_SRVOK))serverOK = true;
      }else{
        trials++;
      }
    serverResponse.clear();
    chkDoc.clear();
    }
    if (trials>30){
      oled.clear();
      noDispChange=false;
      oled.set2X();
      oled.println(" Server\n  Lost");
      oled.println("Listening");
      bool find_server = findServer(false);
    }
  }
  //clear out any packets in the rx buffer
  while(wifiUDPclient.parsePacket()){
    uint8_t jnkBuffer[100];
    int len = wifiUDPclient.read(jnkBuffer,100);
  }
  oled.setCursor(120,7);
  oled.clearToEOL();
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

  bool recLed = true;
  if (serverOK){
    
    //Serial.println("ServerOK");
    //Data Collect, process and Send Tasks....
    //xTaskCreatePinnedToCore(function,pcname,stackdepth,pvparameters,priority...)
    xTaskCreatePinnedToCore(sampleAccelTask,"SampAccelTask",9100,NULL,20,SampleA_Task_Handle,1);
    xTaskCreatePinnedToCore(sampleHallTask,"SampleHallTask",9100,NULL,20,SampleH_Task_Handle,1);

    while(active && WiFi.status()==WL_CONNECTED){//The main data collection loop
      
      //Blink red led while session active
      if ((millis() - blinkTime)>=1000){
        blinkTime = millis();
        recLed = !recLed;
        if (recLed) ledRedOn();
        if (!recLed) ledRedOff();
      }

      loopStartTime=millis();
      delay(100);

      if ((loopStartTime - tTime)>=1000){
        if (digitalRead(REED_SWITCH)){
          active = false;
          tTime = loopStartTime;
          xSemaphoreTake(UDPMutexA, portMAX_DELAY);
          xSemaphoreTake(UDPMutexH, portMAX_DELAY);
          endSession();
          ledRedOff();
        }else (tTime = loopStartTime);
      }
    }//End of While(Active)
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
  calDoc[KEY_BIOSAVZ] = BiotSavartCalConstantZ;

  JsonArray calX = calDoc.createNestedArray(KEY_HCALX);
  JsonArray calY = calDoc.createNestedArray(KEY_HCALY);
  JsonArray calZ = calDoc.createNestedArray(KEY_HCALZ);

  for (int index = 0; index<NUMSENSORS;index++){
      calX.add(sensors[index].avgX);
      calY.add(sensors[index].avgY);
      calZ.add(sensors[index].avgZ);
  }
  int jsize = calDoc.memoryUsage();
    Serial.print("calDoc Mem Usage: ");
    Serial.println(jsize,DEC);
  uint8_t msgPackBuffer[UDPBUFSIZE];
  //Serial.print(calDoc.memoryUsage(),DEC);
  uint16_t bufLen = serializeMsgPack(calDoc,msgPackBuffer);
  //Serial.println(bufLen);
  wifiUDPclient.beginPacket(srvAddress,srvPort);
  wifiUDPclient.write(msgPackBuffer,bufLen);
  wifiUDPclient.endPacket();
  calDoc.clear();
}



void IRAM_ATTR scanHallMatrix(){
      //Take one reading from each of our sensors
  //int st = micros();
  for(int sIndex = 0; sIndex <NUMSENSORS; sIndex++){
    readALS31300ADC(sIndex);
  }
  //int end = micros();
  //Serial.print("Matrix Scan Time: ");
  //Serial.println((end-st), DEC);
  //3 to 4 ms with clocks 1000000 and 30,30 set reg
  //4 to 5 ms with clocks 1000000 and default regs
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
  currSensor->totZ = currSensor->totZ - currSensor->sampZ[currSensor->sampIndex];
        
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
  currSensor->sampZ[currSensor->sampIndex] = SignExtendBitfield(((value0x28 >> 4) & 0x0FF0) | ((value0x29 >> 8) & 0x0F), 12);

  //Update moving average total
  currSensor->totX= currSensor->totX + currSensor->sampX[currSensor->sampIndex];
  currSensor->totY= currSensor->totY + currSensor->sampY[currSensor->sampIndex];
  currSensor->totZ= currSensor->totZ + currSensor->sampZ[currSensor->sampIndex];
  
  currSensor->sampIndex = currSensor->sampIndex +1;
               
  //Ring buffer index wrap-around
  currSensor->sampIndex=currSensor->sampIndex%HALLAVERAGINGSAMPLES;

  //Update average value for sensor (TRUNCATED TO INTEGERS....)
  currSensor->avgX= (currSensor->totX) / (int16_t)HALLAVERAGINGSAMPLES;
  currSensor->avgY= (currSensor->totY) / (int16_t)HALLAVERAGINGSAMPLES;
  currSensor->avgZ= (currSensor->totZ) / (int16_t)HALLAVERAGINGSAMPLES;

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

void powerOffSensors(){
  delay(50); //Data sheet indicates 50ms required for EEPROM WRITE to finish
  digitalWrite(CTL_VDIVIDER, HIGH); //If this is left floating, will attempt to keep chips powered....bad!
  digitalWrite(_CTL_CHA1_PWR, HIGH);
  delay(300); //TUNING - Scope to find actual fall time of power rail.....
}
void powerOnSensors(bool useEEPROM){
  if (useEEPROM){//Use addresses stored in EEPROM
    //Switch N-Fet out of circuit and allow divider taps to float up to VCC
    digitalWrite(CTL_VDIVIDER, LOW);
  }else{//Use Addresses assigned via V-Divider
    //Enable voltage divider for address pins
    digitalWrite(CTL_VDIVIDER, HIGH);
  }
  delay(33);//TUNING - Scope to find actual transition Time.
  digitalWrite(_CTL_CHA1_PWR, LOW); //Turn on Power supply to sensors
  delay(200); //TUNING - Scope to find actual rise time and stabilization period from datasheet.
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
    int jsize = hallDoc.memoryUsage();
    Serial.print("HallDoc Mem Usage: ");
    Serial.println(jsize,DEC);
    int mSize = serializeMsgPack(hallDoc,hMsgBuffer);
    xSemaphoreGive(hMutex);
    hSendTaskUDP.beginPacket(srvAddress,srvPort);
    hSendTaskUDP.write(hMsgBuffer,mSize);
    hSendTaskUDP.endPacket();
    vTaskDelete(NULL);
}

void IRAM_ATTR packetizeAndSendA( void *pvParameters){
    xSemaphoreTake(aMutex, portMAX_DELAY);
    int jsize = accelDoc.memoryUsage();
    Serial.print("AccelDoc Mem Usage: ");
    Serial.println(jsize,DEC);
    int mSize = serializeMsgPack(accelDoc,aMsgBuffer);
    xSemaphoreGive(aMutex);
    aSendTaskUDP.beginPacket(srvAddress,srvPort);
    aSendTaskUDP.write(aMsgBuffer,mSize);
    aSendTaskUDP.endPacket();
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
      xTaskCreate(packetizeAndSendA,"PackNSendA",9000,NULL,22,NULL);
      delayMicroseconds(100);
      xSemaphoreTake(aMutex, portMAX_DELAY);
      /*Serial.print("AccelDoc.memoryUsage()  ");
      Serial.print(accelDoc.memoryUsage(),DEC);
      Serial.print("of ( ");
      Serial.print(accelDoc.capacity(),DEC);
      Serial.println(")");*/
      
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
      //Serial.println(dataReady,DEC);
    }
    if (!dataReady){
      //vTaskDelay(ACCELSAMPPERIOD);
      xTaskDelayUntil(&xLastRunTime, xTaskDelayTimeA);
      //Deadlines are missed but I'm not sure we care in the end
    }else accelDoc[KEY_REALSTOP]= millis();
  }
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
        xTaskCreate(packetizeAndSendH,"PackNSendH",9000,NULL,22,NULL);
        
        delayMicroseconds(100); //Wait for previous task to get established
        xSemaphoreTake(hMutex, portMAX_DELAY);
        /*Serial.print("HallDoc.memoryUsage()  ");
        Serial.print(hallDoc.memoryUsage(),DEC);
        Serial.print("of ( ");
        Serial.print(hallDoc.capacity(),DEC);
        Serial.println(")");*/

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
          Hz[hallPacketIndex] = hallDoc.createNestedArray(HzLabel[hallPacketIndex]);

          //Populate the nested arrays with the actual sensor values
          for(int sIndex = 0; sIndex <(NUMSENSORS); sIndex++){
            Hx[hallPacketIndex].add(sensors[sIndex].avgX );
            Hy[hallPacketIndex].add(sensors[sIndex].avgY );
            Hz[hallPacketIndex].add(sensors[sIndex].avgZ );
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
  digitalWrite(_CTL_ACCEL_PWR, LOW);//Accel On
  delay(50);
  digitalWrite(_CTL_VCC3V3,LOW); //3v3 on
  delay(50);
  digitalWrite(CTL_VDIVIDER, LOW);//Divider off, use EEPROM addresses
  delay(25);
  digitalWrite(_CTL_CHA1_PWR, LOW);//CHA1 on
  delay(50);
  digitalWrite(_CTL_CHA2_PWR, LOW);//CHA2 On
  delay(50);

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
  startDoc[KEY_HALLUSESZ] = 0;
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
    uint8_t buffer[100];
    StaticJsonDocument<100> serverResponse;
    int pSize = wifiUDPclient.parsePacket();
    if (pSize){
      int len = wifiUDPclient.read(buffer,100);
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
    oled.setCursor(62,6);
    oled.print("#:");
    oled.print(mySession,DEC);
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
  oled.setCursor(0,5);
  uint8_t dotCount = 0;
  while(!digitalRead(REED_SWITCH)){ //Wait for reed switch to be in place before gathering data
    sTime = millis();
    if (sTime%1000 == 0){
      oled.print(".");
      dotCount++;
      delay(10);
    
    }
    if (dotCount==21){
      oled.setCursor(0,5);
      oled.clearToEOL();
      dotCount=0;
    }
  }
  delay(100);
  
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