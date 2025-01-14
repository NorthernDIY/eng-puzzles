/*
 *  Written by David Stewart for ECE4600 G15 2021/2022
 *
 * 
 * 
 *  Portions of this code are based on VintLabs 3D-Hall available at https://github.com/vintlabs/3D-Hall
 *  Portions of this code are based on ALS31313 example code provided by Allegro Microsystems:
 *    Written by K. Robert Bate, Allegro MicroSystems, LLC.
 * 
 */

//Board:  ESP32 Nano32S
#define SIMULATE24SENSORS 0

#define VERSIONSTRING "V1.1-MAR-7-2022-EarlyDemoDay"

//Wifi Credentials
#define G15_SSID "ECE4600_G15"
#define G15_PASSWORD "ECE4600G15"

//Server Endpoints
#define G15_RESETURL "http://192.168.1.51:5000/r"//Just gives initial configuration data such as block size of data to be received during a session

#define G15_CALURL "http://192.168.1.51:5000/caldata"//Endpoint to post calibration data to, also initiates a new session

#define G15_HALLURL "http://192.168.1.51:5000/h"
#define G15_ACCURL "http://192.168.1.51:5000/a"

#define REED_SWITCH_ACTIVATION_TIME 200  //Switch debounce timeout

//Hall Sensor Matrix Properties
#define HALLAVERAGINGSAMPLES 40 //Number of samples in moving average
#define NUMSENSORS 24 //Should be 24 in final project
#define HALLSAMPPERIOD 10 //5.5ms is absolute min delay between samples
#define HALLREPORTTIME 50 //Report time interval
#define BLOCKSIZEHALL 1  //10 may have fragmentation issues that cause hiccups!

//Accelerometer Properties
#define ACCELSAMPPERIOD 10
#define BLOCKSIZEACCEL 4

//ALS31313 Hall Sensor parameters (from datasheet)
#define EEPROMDEFAULT 0b00000000000000000000001111100000
#define EEPROMADDRESSMASK 0b11111111111111100000001111111111
#define CAM_REGISTER 0x35
//#define CAM_KEY 0x2C41354 //Was incorrect in example code //ALS31313 Sensors
#define CAM_KEY 0x2C413534 //ALS31300 Sensors

//MCPWM LED CONTROL Macros - Too short to need a seperate function, too long to type out in full each time
#define haltGreenLed() MCPWM0.timer[0].mode.start=0
#define solidGreenLed() mcpwm_set_signal_high(MCPWM_UNIT_0,MCPWM_TIMER_0, MCPWM_OPR_A)
#define offGreenLed() mcpwm_set_signal_low(MCPWM_UNIT_0,MCPWM_TIMER_0, MCPWM_OPR_A)

#define haltRedLed() MCPWM1.timer[0].mode.start=0
#define solidRedLed() mcpwm_set_signal_high(MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_OPR_A)
#define offRedLed() mcpwm_set_signal_low(MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_OPR_A)

//To speed up final project computational speed, uncomment the following!
//This will require additional Flash/Ram, but can provide substantial speed increases.
//#pragma GCC optimize ("-O2")

//Libraries required
#include <Arduino.h>
#include <Wire.h> //Provides I2C
#include <WiFi.h>
#include <HTTPClient.h> //Used for HTTP Comms to server
#include <ArduinoJson.h> //Data sent is serialized in JSON, this library gives us easy access to that data type
#include "SparkFun_LIS2DH12.h"//Accelerometer embedded in stylus for tremor detection

//Libraries to enable Output of LOW FREQ PWM Using MCPWM Peripheral
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/mcpwm.h"

//Actual C code files C:\Users\---username---\.platformio\packages\framework-espidf\components\driver
#include "driver/i2c.h" //Needed to allow use of I2C peripheral settings to actually reach 1MHz!
TwoWire I2C_HS = TwoWire(0);//HS I2C (Hall Sensors)
TwoWire I2C_MS = TwoWire(1);//MS I2C (Accelerometer)

SPARKFUN_LIS2DH12 accel;
#define ACCEL_ADDRESS 0x19

const char* ssid = G15_SSID;
const char* password = G15_PASSWORD;
String hallServerName = G15_HALLURL;
String accelServerName = G15_ACCURL;
String resetServerName = G15_RESETURL;
//String sessionStartServerName = G15_SESSIONSTARTURL;
String caldataServerName = G15_CALURL;

// Return values of endTransmission in the Wire library
#define kNOERROR 0
#define kDATATOOLONGERROR 1
#define kRECEIVEDNACKONADDRESSERROR 2
#define kRECEIVEDNACKONDATAERROR 3
#define kOTHERERROR 4

//Debugging Things
int skipCountHall = 0;  // Keep track of number of times through main loop without reporting
int skipCountAccel = 0;  // Keep track of number of times through main loop without reporting


//Moving Average Struct to store Hall sensor readings
typedef struct{
  uint8_t sampIndex;
  int sampX[HALLAVERAGINGSAMPLES];
  int sampY[HALLAVERAGINGSAMPLES];
  int sampZ[HALLAVERAGINGSAMPLES];
  float avgX,avgY,avgZ;//Stored as int using multiplier 10000
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
int hallPacketIndex = 0;
int accelPacketIndex = 0;

//Number of matrix scans to bundle into a packet
const uint8_t hallBlockSize = BLOCKSIZEHALL;
const uint8_t accelBlockSize = BLOCKSIZEACCEL;

//Json stores hallBlockSize sample periods worth of data.
DynamicJsonDocument hallDoc((1536 + 64) * hallBlockSize);//TUNING - Recalculate packet size based on new Hx# values

JsonArray Hx[hallBlockSize];
JsonArray Hy[hallBlockSize];
JsonArray Hz[hallBlockSize];

JsonArray Ax,Ay,Az;

WiFiClient client;

//We generate these once below instead of sprintf'ing our way to new strings each packet
//This saves a lot of time
char HxLabel[hallBlockSize][6];
char HyLabel[hallBlockSize][6];
char HzLabel[hallBlockSize][6];

//Json stores accelBlockSize sample periods worth of data
DynamicJsonDocument accelDoc((64 + 4) * accelBlockSize);//TUNING - Recalculate packet size based on new Ax# values

//Tracking timestamps of last sample, and last report.  TODO - CONSIDER Move to Timer interrupt based sampling
unsigned long loopStartTime, lastHallSampleTime, lastHallReportTime, lastAccelSampleTime, lastAccelReportTime;

//This is the power up addresses of the sensors when using the voltage divider inputs to ADR0/1
//Repeated for convenience across 2 channels
uint8_t FactoryPowerUpAddresses[NUMSENSORS] = {96,97,98,99,100,101,102,103,104,105,106,107,96,97,98,99,100,101,102,103,104,105,106,107};

//Addresses of hall sensors if they have been programmed
const int sensorAddresses[] = {12,22,11,21,14,24,13,23,16,26,15,25,32,42,31,41,34,44,33,43,36,46,35,45};
const int calSensorAddress = 31;
const int calSensorIndexLocation = 14;//Location in the array above...used to reference sensor data for BiotSavart Const Calculation
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
void powerOffSensors();
void powerOnSensors_useEEPROMAddressses();
void powerOnSensors_useDividerAddresses();

void initSensor(uint8_t index);
void enterCam(uint8_t busAddress, uint32_t magicKey);
void setup();
void IRAM_ATTR loop();
void IRAM_ATTR sampleAccelerometer();
void IRAM_ATTR scanHallMatrix();
void IRAM_ATTR buildSendHallPacket(uint32_t loopStartTime);
void IRAM_ATTR buildSendAccelPacket(uint32_t loopStartTime);
void IRAM_ATTR readALS31300ADC(uint8_t index);
uint16_t read(int busAddress, uint8_t address, uint32_t& value);
uint16_t write(int busAddress, uint8_t address, uint32_t value);
long IRAM_ATTR SignExtendBitfield(uint32_t data, int width);
void setupNoCPULEDBlink(uint8_t pin1, uint8_t pin2); //Setup the MCPWM Peripheral to control led blinking
void blinkRedLed();
void blinkGreenLed();
void initI2CBusses();
void beginSession();
void sendCalData();
void clearAccelPacketBuffer();
void clearHallPacketBuffer();

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
  //Setup pin directions   
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

  digitalWrite(IND_LED1, HIGH);
  digitalWrite(IND_LED2, HIGH);
  digitalWrite(IND_LED3, HIGH);

 //Turn on main power and Channels 1 & 2 - Divider off = assume eeprom is setup already
  digitalWrite(_CTL_VCC3V3,LOW); //3v3 on
  digitalWrite(CTL_VDIVIDER, LOW);
  delay(10);
  digitalWrite(_CTL_CHA1_PWR, LOW);//CHA1 on
  digitalWrite(_CTL_CHA2_PWR, LOW);//CHA2 On
  digitalWrite(_CTL_ACCEL_PWR, LOW);//Accel On
  delay(250);

  digitalWrite(IND_LED1, LOW);
  digitalWrite(IND_LED2, LOW);
  digitalWrite(IND_LED3, LOW);
  
  //setupNoCPULEDBlink(IND_LED1, IND_LED2); //Setup the MCPWM peripherals to handle low speed led blinking
  //blinkGreenLed();

  // Initialize the serial port
  Serial.begin(230400);
  int freq = ESP.getCpuFreqMHz();
  Serial.print("\nSystem Frequency: ");
  Serial.println(freq);
  Serial.print("Connecting to WiFi:");
  Serial.print(ssid);
  
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(250);
    Serial.print(".");
  }
  Serial.print("Connected!\nLocal IP: ");
  Serial.println(WiFi.localIP());
  
  initI2CBusses(); // Initialize the I2C communication ports

  //Initialize each Hall sensor -- Enters CAM mode and sets up Full Loop Register 
  for (uint8_t index = 0; index<NUMSENSORS; index++){
    initSensor(index);
  }

  //Initialize Accelerometer
  accel.begin(ACCEL_ADDRESS, I2C_MS);
  accel.setDataRate(LIS2DH12_ODR_400Hz);
  //accel.setSensitivity();

  //Development Info
  Serial.printf("SETTINGS:,SamplePeriod:,%dms,ReportRate:,%dms,NumSensors:,%d,AvgSamples:,%d\n", HALLSAMPPERIOD,HALLREPORTTIME,NUMSENSORS,HALLAVERAGINGSAMPLES);

  //TODO - Move to separate function.  Nothing below is a variable so no need to pass any data; Return value can be httpResponseCode  
  //Send reset to server along with block size information + other interesting things
  HTTPClient http;
  StaticJsonDocument<2048> startDoc;
  startDoc["Version"] = VERSIONSTRING;
  //startDoc["HallID"]="ALS31313KLEATR-500";
  startDoc["HallID"]="ALS31300EEJASR-500";
  startDoc["H_Samp_Int"]=HALLREPORTTIME;//This is the sample rate that we sample the AVERAGE, the actual sample rate is higher
  startDoc["NumSens"] = NUMSENSORS;
  startDoc["SIMULATED24"]= SIMULATE24SENSORS;
  startDoc["Hbs"] = hallBlockSize;
  startDoc["AccelID"] = "LIS2DH12";
  startDoc["A_Samp_Int"] = ACCELSAMPPERIOD;
  startDoc["Abs"] = accelBlockSize;
  startDoc["Core Freq"]= freq;
  http.begin(client, resetServerName);
  http.addHeader("Content-Type", "application/json");
  String stringifiedJson;
  serializeJson(startDoc, stringifiedJson);
  int httpResponseCode =http.POST(stringifiedJson);
  uint8_t retryInitialPostCount = 0;
  while (httpResponseCode!=200 && retryInitialPostCount<20){
    httpResponseCode = http.POST(stringifiedJson);
    delay(500);
    retryInitialPostCount++;
  }
  if (httpResponseCode != 200){
    //offGreenLed();
    //blinkRedLed();
    Serial.println("Unable to reach Raspberry Pi Server, Make sure it's running!\nHalting!");
    while(1);
  }
  http.end();
  startDoc.clear();
  
  //Initialize Global variables used in session loop //TODO - Move to local variables .....if reasonable to do so.  Likely to need lots of pass by value function calls...
  lastHallSampleTime = 0;
  lastHallReportTime = 0;
  lastAccelSampleTime = 0;
  lastAccelReportTime = 0;
  
  //Generate our Hall Sensor Keys in advance, avoids sprintf every packet (speedup!)
  for (int index = 0; index<hallBlockSize;index++){
    sprintf(HxLabel[index], "Hx%u", index);
    sprintf(HyLabel[index], "Hy%u", index);
    sprintf(HzLabel[index], "Hz%u", index);
  }
  
  //For the BiotSavart Constant
  digitalWrite(IND_LED3, HIGH); //Turn on Yellow LED, Indicate waiting!
  while(!digitalRead(REED_SWITCH)); //Wait for reed switch to be in place before gathering data
  uint32_t sTime = millis();
  uint32_t cTime=0;
  int count = 0;
  while(count<50){
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
  digitalWrite(IND_LED3, LOW);
  
  Serial.println("G15 Instrumented Puzzle Initialized...\nReady state starts in 1 Sec");
  delay(1000);
  //solidGreenLed();//Everything is ready to record data!
}

//Main loop just waits for reed switch to activate
void loop()
{
  uint32_t tTime = millis();
  bool Triggered = false;
  while(1){
    if (!digitalRead(REED_SWITCH) && !Triggered){
      Triggered = true;
      tTime = millis();
    }else if(digitalRead(REED_SWITCH)) {
      Triggered = false;
      tTime = 0;
    }
    if (Triggered && ((millis() - tTime)>=REED_SWITCH_ACTIVATION_TIME)){
      beginSession();
    }
  }//End of While(1)
}//End of Loop()

//Gathers and sends CalData before 
void IRAM_ATTR beginSession(){
  hallTimeStamp = 0;
  accelTimeStamp = 0;
  lastHallSampleTime = 0;
  lastHallReportTime = 0;
  lastAccelSampleTime = 0;
  lastAccelReportTime = 0;
 
  skipCountHall =0;//Reset the skip counter
  hallPacketIndex = 0; //Reset the packet index for our next data block
  skipCountAccel =0;
  accelPacketIndex = 0;
  //RESET packet indexes

  sendCalData();//This could change if for example, maze is moved after powerup
  //Populate the initial time stamps, avoids repeated if logic in buildSendPacket
  hallDoc["T"] = hallTimeStamp;
  accelDoc["T"] =accelTimeStamp;
  //To avoid blank first Accel packet
  Ax = accelDoc.createNestedArray("X");
  Ay = accelDoc.createNestedArray("Y");
  Az = accelDoc.createNestedArray("Z");
  uint32_t tTime = 0;
  //blinkRedLed();
  bool Active = true;//state variable toggled by reed switch
  
  while(Active){//The main data collection loop
      loopStartTime = millis();
      //Sample accelerometer if it is time to do so again.
      if ((loopStartTime - lastAccelSampleTime)>ACCELSAMPPERIOD){
        buildSendAccelPacket(loopStartTime);
        lastAccelSampleTime = loopStartTime;//Update the last SAMPLED time
      }else{
        skipCountAccel++;//Wasn't time so increment the skipped counter
      }
      
      loopStartTime = millis();//Regrab the time as the above might actually take time to complete!
      
      //Sample Hall Matrix if it is time to do so again.
      if ((loopStartTime-lastHallSampleTime)>HALLSAMPPERIOD){
        lastHallSampleTime = loopStartTime; //update the last SAMPLED time
        scanHallMatrix();
        //Serial.printf("loopStart: %d\tlastHallSamp: %d\tlastHallRep: %d\n",loopStartTime, lastHallSampleTime, lastHallReportTime);
        if ((loopStartTime-lastHallReportTime)>(HALLREPORTTIME))buildSendHallPacket(loopStartTime);
      }else{
        skipCountHall++; //Wasn't time so increment skipped counter
      }
      if ((loopStartTime - tTime)>=1000){
        if (digitalRead(REED_SWITCH)&&hallPacketIndex){
          Active = false;
          tTime = loopStartTime;
          offRedLed();
          clearHallPacketBuffer();
          clearAccelPacketBuffer();
        }else (tTime = loopStartTime); //Forces sessions to be increments of 1 Second long
      }
    }//End of While(Active)
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
    if ((millis()-sTimer) >=HALLSAMPPERIOD){
      sTimer = millis();
      scanHallMatrix();
      count++;
    }     
   }
  HTTPClient http;
  //StaticJsonDocument<2560> calDoc;
  DynamicJsonDocument calDoc(2816);
  calDoc["BSX"] = BiotSavartCalConstantX;
  calDoc["BSY"] = BiotSavartCalConstantY;
  calDoc["BSZ"] = BiotSavartCalConstantZ;

  JsonArray calX = calDoc.createNestedArray("H_CAL_X");
  JsonArray calY = calDoc.createNestedArray("H_CAL_Y");
  JsonArray calZ = calDoc.createNestedArray("H_CAL_Z");
  for (int index = 0; index<NUMSENSORS;index++){
      calX.add(sensors[index].avgX);
      calY.add(sensors[index].avgY);
      calZ.add(sensors[index].avgZ);
  }

  http.begin(client, caldataServerName);
  http.addHeader("Content-Type", "application/json");
  String stringifiedJson;
  serializeJson(calDoc, stringifiedJson);
  //Send it.
  int httpResponseCode = http.POST(stringifiedJson);
  http.end();
  //Serial.println("New Session!...Cal Data Sent");
  //Serial.println(stringifiedJson);
  calDoc.clear();
}

void IRAM_ATTR buildSendAccelPacket(uint32_t loopStartTime){

    //if the packet index is the size of our blocksize, then it is time to transmit
    //at T=0, there are no samples to report hence, accelTimeStamp>accelBlockSize
    if ((accelPacketIndex%accelBlockSize)==0  && accelPacketIndex>0){
      HTTPClient http;
      accelDoc["SC"] = skipCountAccel; //Performance metrics, how many main loops we skipped without sampling.
      accelPacketIndex = 0;//reset the sample index within the packet.
            
      http.begin(client, accelServerName);
      http.addHeader("Content-Type", "application/json");//Might not actually need this!
      
      //Convert the json document to a string for transmission
      String stringifiedJson;
      serializeJson(accelDoc, stringifiedJson);
      
      //Send it
      int httpResponseCode = http.POST(stringifiedJson);//TODO - Use response code to indicate issues!!!
      http.end();
      
      clearAccelPacketBuffer();
    }else{
      Ax.add(accel.getX());
      Ay.add(accel.getY());
      Az.add(accel.getZ());
      accelPacketIndex++;
    }
}

void IRAM_ATTR clearAccelPacketBuffer(){

      accelPacketIndex = 0;//reset the sample index within the packet.
      accelDoc.clear();//Clear out old information in the Json Structure.
      skipCountAccel=0;//Reset the skip counter
      accelTimeStamp +=(accelBlockSize*ACCELSAMPPERIOD);//Increment the timestamp by the block size
      accelDoc["T"] =  accelTimeStamp;//Prepend the timestamp of the first next sample to the JSON document
      lastAccelReportTime=loopStartTime;//Record the last Reported Time
      
      //Recreate the nested arrays for next time
      Ax = accelDoc.createNestedArray("X");
      Ay = accelDoc.createNestedArray("Y");
      Az = accelDoc.createNestedArray("Z");
}

void IRAM_ATTR clearHallPacketBuffer(){
  hallDoc.clear();
  skipCountHall =0;//Reset the skip counter
  hallPacketIndex = 0; //Reset the packet index for our next data block
  hallTimeStamp +=(hallBlockSize*HALLREPORTTIME); //Increment the timestamp by the blocksize
  hallDoc["T"] =  hallTimeStamp; //Set the starting timestamp of the next packet
}
void IRAM_ATTR buildSendHallPacket(uint32_t loopStartTime){

  /*When the packet is sent, we prepend the timestamp of the Next First sample
  * to the new JSON document.
  *  Time stamps of full packets:
  *    0
  *    (blockSizeHall*HALLREPORTPERIOD * 1)
  *    (blockSizeHall*HALLREPORTPERIOD * 2)
  *     ^
  *     v
  *     T-(blockSizeHall*HALLREPORTPERIOD)
  *     T
  *     T+(blockSizeHall*HALLREPORTPERIOD)
  */
  
  //If the packet is full, we need to send it.
  if ((hallPacketIndex % hallBlockSize)==0 && hallPacketIndex>0){
    HTTPClient http;
    hallDoc["SC"] = skipCountHall; //Performance Metric
    http.begin(client, hallServerName);
    http.addHeader("Content-Type", "application/json");
    
    //Convert the JSON document to a string for transmission
    String stringifiedJson;
    serializeJson(hallDoc, stringifiedJson);
    
    //Send it.
    int httpResponseCode = http.POST(stringifiedJson);
    http.end();
        
    //Clear out old data in the JSON document
    //Since packet format is consistent it may be possible to just overwrite everything but that will end up with FRAGMENTATION ISSUES probably
    clearHallPacketBuffer();
    
    }else{//Add, we can add the hall sensor samples to the packet
      
      //Add the nested arrays to the JSON doc for this sample
      Hx[hallPacketIndex] = hallDoc.createNestedArray(HxLabel[hallPacketIndex]);
      Hy[hallPacketIndex] = hallDoc.createNestedArray(HyLabel[hallPacketIndex]);
      Hz[hallPacketIndex] = hallDoc.createNestedArray(HzLabel[hallPacketIndex]);

      //Populate the nested arrays with the actual sensor values
      for(int sIndex = 0; sIndex <(NUMSENSORS); sIndex++){
        Hx[hallPacketIndex].add(sensors[sIndex].avgX);
        Hy[hallPacketIndex].add(sensors[sIndex].avgY);
        Hz[hallPacketIndex].add(sensors[sIndex].avgZ);
      }

      lastHallReportTime=loopStartTime;  //Update the last REPORTED TIME
      hallPacketIndex++;
    }
}

void IRAM_ATTR scanHallMatrix(){
      //Take one reading from each of our sensors
  for(int sIndex = 0; sIndex <NUMSENSORS; sIndex++){
    readALS31300ADC(sIndex);
  } 
}


// readALS31300ADC
// Assumes hall sensor is in full loop mode.
void IRAM_ATTR readALS31300ADC(uint8_t index){
//void readALS31300ADC(uint8_t index){
  MASensor* currSensor = &sensors[index];  //This curses us with -> below.  TODO FIX THIS UP
  
  //Subtract fromt the total, the last value saved at this location in our ring buffer
  currSensor->totX = currSensor->totX - currSensor->sampX[currSensor->sampIndex];
  currSensor->totY = currSensor->totY - currSensor->sampY[currSensor->sampIndex];
  currSensor->totZ = currSensor->totZ - currSensor->sampZ[currSensor->sampIndex];
        
  // Start the read and request 8 bytes
  // which are the contents of register 0x28 and 0x29
  I2C_HS.requestFrom(currSensor->address, 8);
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
  //if (currSensor->sampIndex>=HALLAVERAGINGSAMPLES) currSensor->sampIndex = 0;
  currSensor->sampIndex=currSensor->sampIndex%HALLAVERAGINGSAMPLES;//This should be computationally less intensive.

  //Update average value for sensor
  currSensor->avgX= (float)(currSensor->totX) / (float)HALLAVERAGINGSAMPLES;
  currSensor->avgY= (float)(currSensor->totY) / (float)HALLAVERAGINGSAMPLES;
  currSensor->avgZ= (float)(currSensor->totZ) / (float)HALLAVERAGINGSAMPLES;
}


//
// read
//
// Using I2C, read 32 bits of data from the address on the device at the bus address
// (This code is from VintLabs)
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
// (This code is from VintLabs)
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
// (This code is from VintLabs)
long IRAM_ATTR SignExtendBitfield(uint32_t data, int width)
//long SignExtendBitfield(uint32_t data, int width)
{
    long x = (long)data;
    long mask = 1L << (width - 1);

    if (width < 32)
    {
        x = x & ((1 << width) - 1); // make sure the upper bits are zero
    }

    return (long)((x ^ mask) - mask);
}

void powerOffSensors(){
  delay(50); //Data sheet indicates 50ms required for EEPROM WRITE to finish
  digitalWrite(CTL_VDIVIDER, HIGH); //If this is left floating, will attempt to keep chips powered....bad!
  digitalWrite(_CTL_CHA1_PWR, HIGH);
  delay(500); //TUNING - Scope to find actual fall time of power rail.....
}
void powerOnSensors_useEEPROMAddressses(){
  //Switch N-Fet out of circuit and allow divider taps to float up to VCC
  digitalWrite(CTL_VDIVIDER, LOW);
  delay(33);//TUNING - Scope to find actual transition Time.
  
  //Enable Main power rail and wait for stabilzation
  digitalWrite(_CTL_CHA1_PWR, LOW);
  delay(500);//TUNING - Scope to find actual rise time and stabilization period from datasheet
}
void powerOnSensors_useDividerAddresses(){
  //Enable voltage divider for address pins
  digitalWrite(CTL_VDIVIDER, HIGH);
  delay(33);//TUNING - Scope to find actual transition Time.
  
  digitalWrite(_CTL_CHA1_PWR, LOW); //Turn on Power supply to sensors
  delay(200); //TUNING - Scope to find actual rise time and stabilization period from datasheet.
}


//Enter CUSTOMER ACCESS MODE on hall sensor, allows you to write to registers
//to configure hall sensor as well as set I2C address in EEPROM
void enterCam(uint8_t busAddress, uint32_t magicKey){
  uint16_t wireError = write(busAddress, CAM_REGISTER, magicKey);
  if (wireError != kNOERROR){
    Serial.print("Error while trying to enter customer access mode for sensor @0x");
    Serial.print(busAddress,HEX);
    Serial.print("\terror = ");
    Serial.println(wireError);
  }else{
    //Serial.print("\t@0x");
    //Serial.print(busAddress,HEX);
    //No actual way to check status of CAM_REGISTER so we can only assume that if the i2c transmission was ok, it worked!
    //Serial.println(" Entered C.A.M Successfully");
  }  
}

void initSensor(uint8_t index){
  enterCam(sensorAddresses[index], CAM_KEY);
  
  //TODO CLEAN UP CODE AND JUST USE index below!
  uint8_t busAddress = sensorAddresses[index];
  //Disable Loop Mode //Should be disabled by default
  
  uint32_t value0x27;
  // Read the register the I2C loop mode is in
  uint16_t error = read(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to read the ALS31300. error = ");
        Serial.println(error);
    }

    // I2C loop mode is in bits 2 and 3 so mask them out
    
    //We want full loop mode so 3:2 should be 10,  
    value0x27 = (value0x27 & 0xFFFFFFF3) | (0b1000 );
    
    // Write the new values to the register the I2C loop mode is in
    error = write(busAddress, 0x27, value0x27);
    if (error != kNOERROR)
    {
        Serial.print("Unable to write to the ALS31300. error = ");
        Serial.println(error);
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
  //May be useful later, but possible to remove
  sensors[index].address=sensorAddresses[index];
}

/*  setupNoCPULEDBlink
 *  
 *  Setup the MCPWM peripherals for a low frequency PWM output.
 *  This allows us to blink leds without cpu usage entirely.
*/
/*void setupNoCPULEDBlink(uint8_t pin1, uint8_t pin2){
  
  //Borrowed and modified code from MartinL (https://forum.arduino.cc/t/esp32-what-is-the-minimum-pwm-frequency/671077)
  
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pin1);     //Green LED on Unit 0
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, pin2);     //Red Led on Unit 1
  //Green LED
  MCPWM0.clk_cfg.prescale = 199;                // Set the 160MHz clock prescaler to 199 (160MHz/(199+1)=800kHz)
  MCPWM0.timer[0].period.prescale = 199;        // Set timer 0 prescaler to 199 (800kHz/(199+1))=4kHz)
  MCPWM0.timer[0].period.period = 3999;         // Set the PWM Frequency (4kHz/(3999+1) = 1Hz)  
  MCPWM0.channel[0].cmpr_value[0].val = 2000;   // Set the counter compare for 50% duty-cycle
  MCPWM0.channel[0].generator[0].utez = 2;      // Set the PWM0A ouput to go high at the start of the timer period
  MCPWM0.channel[0].generator[0].utea = 1;      // Clear on compare match
  MCPWM0.timer[0].mode.mode = 1;                // Set timer 0 to increment
  MCPWM0.timer[0].mode.start = 0;

  //Red LED
  MCPWM1.clk_cfg.clk_prescale = 199;                // Set the 160MHz clock prescaler to 199 (160MHz/(199+1)=800kHz)
  MCPWM1.timer[0].period.prescale = 199;        // Set timer 0 prescaler to 199 (800kHz/(199+1))=4kHz)
  MCPWM1.timer[0].period.period = 3999;         // Set the PWM Frequency (4kHz/(3999+1)=1Hz)  
  MCPWM1.channel[0].cmpr_value[0].val = 2000;  // Set the counter compare for 50% duty-cycle
  MCPWM1.channel[0].generator[0].utez = 2;      // Set the PWM0A ouput to go high at the start of the timer period
  MCPWM1.channel[0].generator[0].utea = 1;      // Clear on compare match
  MCPWM1.timer[0].mode.mode = 1;                // Set timer 0 to increment
  MCPWM1.timer[0].mode.start = 0;

}
void blinkGreenLed(){
  mcpwm_set_duty_type(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,MCPWM_DUTY_MODE_0);
  MCPWM0.timer[0].mode.start=2;
}
void blinkRedLed(){
  mcpwm_set_duty_type(MCPWM_UNIT_1,MCPWM_TIMER_0,MCPWM_OPR_A,MCPWM_DUTY_MODE_0);
  MCPWM1.timer[0].mode.start=2;
}*/

/*  initI2CBusses
 *  
 *  Setup both I2C busses and adjust SCL clock frequency to match desired
*/

void initI2CBusses(){
  I2C_HS.begin(SDA_HS, SCL_HS);
  I2C_HS.setClock(1000000);//Doesn't actually set clock to 1MHz though... see below!
  /*   >>>According the the ESP32 technical reference manual<<<<<
  * For APB of 80MHz, I2C clock is 80 MHz / (SCL_LOW_LEVEL_CYCLES + SCL_HIGH_LEVEL_CYCLES)
  * SCL_LOW_LEVEL_CYCLES = 1 + I2C_SCL_LOW_PERIOD (Above sets I2C_SCL_LOW_PERIOD = 50)
  * SCL_HIGH_LEVEL_CYCLES = I2C_SCL_FILTER_THRES + I2C_SCL_HIGH_PERIOD (Above sets I2C_SCL_HIGH_PERIOD = 50)
  * >>>By Default, I2C_SCL_FILTER_THRES is set to 7 as per driver/i2c.c<<<
  * 
  * Therefore clock rate SHOULD BE:
  *     80MHz / (51 + 57) = 740 KHz
  * 
  * When measured with LA1034 Logic Analyzer (100MHz Sample rate, Logic Threshold 1.30V)
  *    SCL Period is 1.52uSec, which translates to 658 KHz
  * 
  * Setting LOW/HIGH Periods to 28, 28, SHOULD BE:
  *   80MHz / (29 + 35) = 1.25 MHz
  *
  * When measured with LA1034 Logic Analyzer (100MHz Sample rate, Logic Threshold 1.30V)
  *    SCL Period is  about 1.00uSec, which translates to 1MHz
  * 
  *     10 CLOCK CYCLES: 10.25uSec
  *       570nSec Low (Sample size 1)
  *       460nSec High (Sample size 1)
  */
  
  //esp_err_t i2c_set_period(i2c_port_t i2c_num, int high_period, int low_period)
  i2c_set_period(I2C_NUM_0,28, 28);// 1uSec Clock Period = 1 MHz
  //i2c_set_period(I2C_NUM_0,20, 20);//(Out of spec)

  //MS bus, used for accelerometer
  I2C_MS.begin(SDA_MS, SCL_MS);
  I2C_MS.setClock(435000);//Pretty close to 400KHz as verified by LA1034
}