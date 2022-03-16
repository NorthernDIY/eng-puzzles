/*  ECE4600 G15 ALS31300 Initial Address Programming Firmware
 *   
 *   By David Stewart, ECE4600 G15 2021/2022
 *   
 *   Portions of this code based on the example code provided in the ALS31313 application notes by Allegro Microsystems. 
 * 
 */


//N-Channel disconnects ground from voltage divider circuit.  Causes ADR0/1 to float to VCC.  HIGH = Divider Enabled, LOW = Divider Disabled (Floats to VCC)
#define CTL_VDIVIDER 16

//P-Channel VCC3V3 High Side Switch for everything but the esp32.  Disconnects: VDivider, Vdivider Op Amp Buffer, Accelerometer, Hall Sensors.
#define _CTL_VCC3V3 4

//P-Channel high side switch for sensor channel power.  LOW = Sensor power enabled, HIGH = Sensor power disabled
#define _CTL_CHA1_PWR 17//17
#define _CTL_CHA2_PWR 18//18
#define _CTL_CHA3_PWR 19
#define _CTL_CHA4_PWR 5

//P-Channel high side switch for accelerometer power.  LOW = Sensor power enabled, HIGH = Sensor power disabled
#define _CTL_ACCEL_PWR 27

//Status LEDS
#define IND_LED1 33
#define IND_LED2 25
#define IND_LED3 26

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



//From Application notes, return status values of Wire Transmission attempts
#define kNOERROR 0
#define kDATATOOLONGERROR 1
#define kRECEIVEDNACKONADDRESSERROR 2
#define kRECEIVEDNACKONDATAERROR 3
#define kOTHERERROR 4

#define NUMSENSORS 24


//Datasheet says DEC 960 is default
#define EEPROMDEFAULT 0b00000000000000000000001111100000
#define EEPROMADDRESSMASK 0b11111111111111100000001111111111
#define CAM_REGISTER 0x35
#define CAM_KEY 0x2C413534

#include <Wire.h>

//Power on addresses when voltage divider is enabled and ADR0/1 pins are proportional to VCC.
uint8_t powerUpAddresses[NUMSENSORS] = {96,97,98,99,100,101,102,103,104,105,106,107,96,97,98,99,100,101,102,103,104,105,106,107};
 
//Not really helpful to have, but here for reference
uint8_t factoryEEPROMAddresses[NUMSENSORS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//Desired Addresses
uint8_t newAddresses[NUMSENSORS] = {12,22,11,21,14,24,13,23,16,26,15,25,32,42,31,41,34,44,33,43,36,46,35,45};

bool sensorPresentPU[NUMSENSORS] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
bool sensorPresentNEW[NUMSENSORS] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};


//Function Prototypes
void errorHalt();
uint16_t read(int busAddress, uint8_t address, uint32_t& value);
uint16_t readDBG(int busAddress, uint8_t address, uint32_t& value);
uint16_t write(int busAddress, uint8_t address, uint32_t value);
uint32_t genNewEEPROMAddress(uint32_t currData, uint8_t newAddress, uint32_t& newValue);
bool verifyAddressPresence(uint8_t busAddress);
void enterCam(uint8_t busAddress, uint32_t magicKey);
void powerOff();
void powerOnSensors_useEEPROMAddresses();
void powerOnSensors_useDividerAddresses();
void randomizeNewAddresses();

void setup()
{
    Serial.begin(115200);  
    pinMode(CTL_VDIVIDER, OUTPUT);
    pinMode(_CTL_CHA1_PWR, OUTPUT);
    pinMode(_CTL_CHA2_PWR, OUTPUT);
    pinMode(_CTL_VCC3V3, OUTPUT);
    digitalWrite(_CTL_CHA1_PWR, HIGH);//off
    digitalWrite(_CTL_CHA2_PWR, HIGH);//off
    digitalWrite(_CTL_VCC3V3, HIGH); //off
    digitalWrite(CTL_VDIVIDER, HIGH);//Divider ON
    Serial.println("ALLOFF");
    delay(500);
    digitalWrite(_CTL_VCC3V3, LOW);
    Serial.println("VCC3V3 ON");
    
    powerOffSensors();
    delay(200);
    powerOnSensors_useDividerAddresses();
     
    // Initialize the I2C communication library
    delay(2000);  //Let Serial monitor catch up after upload
    Wire.begin(SDA, SCL);
    Wire.setClock(1000000);    // 1 MHz -ocassional nack errors, probably cause janky wires.
}

void loop()
{
    bool halt = false;
    Serial.println("Verifying power up addresses of sensors:");
    for (int i = 0; i<NUMSENSORS; i++){
        if (i==(NUMSENSORS/2)){
          digitalWrite(_CTL_CHA1_PWR, HIGH);//CHA1 off
          digitalWrite(_CTL_CHA2_PWR, LOW);//CHA2 On
          delay(100);
        }
        sensorPresentPU[i] = verifyAddressPresence(powerUpAddresses[i]);
    }
    
    delay(100);
    digitalWrite(_CTL_CHA2_PWR, HIGH);//CHA2 off
    digitalWrite(_CTL_CHA1_PWR, LOW);//CHA1 On
    delay(100);
        
    Serial.println("Attempting to set new EEPROM I2C Addresses");
    for (int i = 0; i<(NUMSENSORS); i++){
      if (i==(NUMSENSORS/2)){ //Switch channels at 12th sensor (11th for array, zero indexed)
          digitalWrite(_CTL_CHA1_PWR, HIGH);//CHA1 off
          digitalWrite(_CTL_CHA2_PWR, LOW);//CHA2 On
          delay(100);
      }
      if (sensorPresentPU[i]){
        uint32_t currData;
        uint16_t wError;
        uint16_t rError;
        enterCam(powerUpAddresses[i], CAM_KEY);
        delay(50); //Not sure of response time needed here
        rError = read(powerUpAddresses[i], 0x02, currData);
        if (rError ==kNOERROR){
          Serial.print("\t@0x");
          Serial.print(powerUpAddresses[i],HEX);
          Serial.print(" Rd Reg 0x02: 0b");
          Serial.print((byte)(currData >> 24),BIN);
          Serial.print((byte)(currData >> 16),BIN);
          Serial.print((byte)(currData >> 8),BIN);
          Serial.println((byte)(currData),BIN);
        }

        uint32_t newData;
        genNewEEPROMAddress(currData, newAddresses[i], newData);
        delay(10);
        wError = write(powerUpAddresses[i], 0x02, newData);
        if (wError == kNOERROR){
          Serial.print("\t@0x");
          Serial.print(powerUpAddresses[i],HEX);
          Serial.print(" Wr Reg 0x02: 0b");
          Serial.print((byte)(newData >> 24),BIN);
          Serial.print((byte)(newData >> 16),BIN);
          Serial.print((byte)(newData >> 8),BIN);
          Serial.print((byte)(newData),BIN);
        }
        delay(50);//Want to ensure EEPROM Write finished otherwise this will error!
        //Verify write data - not likely needed!
        rError = read(powerUpAddresses[i], 0x02, currData);
        if (newData == currData){
          Serial.println("\t Write Verified!");
        }else{
          Serial.println("\t Write Failed!");
          errorHalt();
        }
      }
    }//Done writing new addressses
    delay(50);
    
    Serial.println("Power Cycling Sensors and Floating Address pins to VCC");
    delay(100);
    digitalWrite(_CTL_VCC3V3, HIGH);//off
    digitalWrite(CTL_VDIVIDER,LOW);//off - float to 3v3
    digitalWrite(_CTL_CHA1_PWR, LOW);//on
    digitalWrite(_CTL_CHA2_PWR, LOW);//on
    delay(200);
    digitalWrite(_CTL_VCC3V3, LOW);//Main power on
    delay(150);
    Serial.println("Verifying NEW addresses of sensors:");
    for (int i = 0; i<(NUMSENSORS); i++){
        Serial.print("@");
        Serial.print(newAddresses[i], HEX);
        Serial.print("? ");
        sensorPresentNEW[i] = verifyAddressPresence(newAddresses[i]);
        
        if (!sensorPresentNEW[i]){
          Serial.println("Addres Verify Failed!");
          errorHalt();
        }
    }
    delay(100);
    Serial.println("WAITING 15 Seconds before restoring defaults.  Disconnect now if you need the new addresses....");
    delay(15000);
    Serial.println("Restoring default EEPROM Values to sensors!");
    for (int i = 0; i<(NUMSENSORS); i++){
        enterCam(newAddresses[i], CAM_KEY);
        delay(10);
        resetSensorEEPROM(newAddresses[i]);
    }
    delay(100);
    Serial.println("Power Cycling Sensors and Setting Address pins proportional to VCC");

    digitalWrite(_CTL_VCC3V3, HIGH);//off
    digitalWrite(_CTL_CHA1_PWR,LOW);//cha1 on
    digitalWrite(CTL_VDIVIDER, HIGH);//vdivider on
    Serial.println("Going again in 30 seconds!");
    delay(30000);
}





//From ALS31313 Example Code
// read
//
// Using I2C, read 32 bits of data from the address on the device at the bus address
//

//Just stops us from spamming eeprom with writes if there's an error -  Don't want to wear them out!
//Also stop the serial ouput from hiding the error through scrolling....
void errorHalt(){
  Serial.println("Halting");
  while(1);
}

uint16_t read(int busAddress, uint8_t address, uint32_t& value)
{
    // Write the address that is to be read to the device
    Wire.beginTransmission(busAddress);
    Wire.write(address);
    int error = Wire.endTransmission(false);

    // if the device accepted the address,
    // request 4 bytes from the device
    // and then read them, MSB first
    if (error == kNOERROR)
    {
        Wire.requestFrom(busAddress, 4);
        value = Wire.read() << 24;
        value += Wire.read() << 16;
        value += Wire.read() << 8;
        value += Wire.read();
    }

    return error;
}
uint16_t readDBG(int busAddress, uint8_t address, uint32_t& value)
{
    // Write the address that is to be read to the device
    Serial.print("ReadDBG - ADR:0x");
    Serial.print(busAddress,HEX);
    Serial.print("\t REG:0x");
    Serial.println(address,HEX);
    
    Wire.beginTransmission(busAddress);
    Wire.write(address);
    int error = Wire.endTransmission(false);

    // if the device accepted the address,
    // request 4 bytes from the device
    // and then read them, MSB first
    uint8_t v1,v2,v3,v4;
    if (error == kNOERROR)
    {
        Wire.requestFrom(busAddress, 4);
        v1 = Wire.read();
        v2 = Wire.read();
        v3 = Wire.read();
        v4 = Wire.read();
        value = v1 << 24;
        value += v2 << 16;
        value += v3 << 8;
        value += v4;
    }
    Serial.print("E?: ");
    Serial.print(error, DEC);
    Serial.print("  Raw Read (MSB to LSB): ");
    Serial.print(v1, BIN);
    Serial.print(" ");
    Serial.print(v2, BIN);
    Serial.print(" ");
    Serial.print(v3, BIN);
    Serial.print(" ");
    Serial.print(v4, BIN);
    Serial.print(" ");
    
    return error;
}
//From ALS31313 Example Code
// write
//
// Using I2C, write 32 bit data to an address to the device at the bus address
//
uint16_t write(int busAddress, uint8_t address, uint32_t value)
{
    // Write the address that is to be written to the device
    // and then the 4 bytes of data, MSB first
    Wire.beginTransmission(busAddress);
    Wire.write(address);
    Wire.write((byte)(value >> 24));
    Wire.write((byte)(value >> 16));
    Wire.write((byte)(value >> 8));
    Wire.write((byte)(value));
    return Wire.endTransmission();
}

uint32_t genNewEEPROMAddress(uint32_t currData, uint8_t newAddress, uint32_t& newValue){
  newValue = currData & EEPROMADDRESSMASK;
  newValue += newAddress <<10;
  //Serial.print("New EEPROM: (DEC) ");
  //Serial.println(newValue,DEC);
}

void enterCam(uint8_t busAddress, uint32_t magicKey){
  uint16_t wireError = write(busAddress, CAM_REGISTER, magicKey);
  if (wireError != kNOERROR){
    Serial.print("Error while trying to enter customer access mode for sensor @0x");
    Serial.print(busAddress,HEX);
    Serial.print("\terror = ");
    Serial.println(wireError);
  }else{
    Serial.print("\t@0x");
    Serial.print(busAddress,HEX);
    //No actual way to check status of CAM_REGISTER so we can only assume that if the i2c transmission was ok, it worked!
    Serial.println(" Entered C.A.M Successfully");
  }  
}
bool verifyAddressPresence(uint8_t busAddress){
     
      Wire.beginTransmission(busAddress);
      uint16_t wireError = Wire.endTransmission();
      if (wireError ==kNOERROR){
        Serial.print("Device presence confirmed @ 0x");
        Serial.println(busAddress, HEX);
        return true;
    }else{
      return false;
    }
}
void powerOffSensors(){
  digitalWrite(CTL_VDIVIDER, HIGH); //If this is left floating, will attempt to keep chips powered....bad!
  digitalWrite(_CTL_CHA1_PWR, HIGH);
  delay(500);
}
void powerOnSensors_useEEPROMAddresses(){
  digitalWrite(CTL_VDIVIDER, LOW);
  delay(50);
  digitalWrite(_CTL_CHA1_PWR, LOW);
  delay(500);
}
void powerOnSensors_useDividerAddresses(){
  digitalWrite(CTL_VDIVIDER, HIGH);
  delay(33);
  digitalWrite(_CTL_CHA1_PWR, LOW);
  delay(500);
}

void randomizeNewAddresses(){
  Serial.print("New Target Addresses: ");
  for (int i =0; i<NUMSENSORS; i++){
      newAddresses[i]=random(100)+1;
      sensorPresentNEW[i] = false;
      Serial.print("0x");
      Serial.print(newAddresses[i],HEX);
      Serial.print(", ");
    }
  Serial.println("");
}

void resetSensorEEPROM(uint8_t busAddress){
  write(busAddress,0x02, EEPROMDEFAULT);
}
