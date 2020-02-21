#include <UbloxGPS.h>
#include <TinyGPS++.h>
#include <RelayXBee.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h> 


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Pin Decleration ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

#define chipSelect BUILTIN_SDCARD //Should highlight if you have teensy 3.5/3.6/4.0 selected
#define smartPin 10               // The data pin on the servo motor. Must have PWM capabilities  -- PCB pin: 2
#define dallasOnePin 33           // Data pin for the first dallas temperature sensor -- PCB pin:
#define dallasTwoPin 34           // Data pin for the second dallas temperature sensor -- PCB pin: 
#define pressureOnePin A1         // Data pin for the first honeywell pressure sensor -- PCB pin:
#define pressureTwoPin A2         // Data pin for the second honeywell pressure sensor -- PCB pin:
#define ubloxSerial Serial1       // Serial communication lines for the ublox GPS -- PCB pins: Serial1
#define xbeeSerial Serial3        // Serial communication lines for the xbee radio -- PCB pins: Serial3
//#define rfd900Serial serialX    //this line should all be commented if you aren't using an RFD900
#define fixLED 39                 // LED to indicate GPS fix
#define xbeeLED 38                // LED to indicate xbee communication
#define statusLED 37              // LED to flash each data cycle
#define uniqueLED 36              // 
#define serialBAUD 9600           // when using arduino serial monitor, make sure baud rate is set to this same value



////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Variable Decleration //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

bool rfd900 = false; // set true if you want this thing to operate with a rfd900 on serialX (declare above)
bool ppod = true; // set true if you want this thing to operate as ppod flight computer
bool magnetBootup = false; // set true if you want to activate flight by waving a magnet over the IMU
bool bootup = false; // for use with the magnetometer


/////////////////////////////// GPS related variables /////////////////////////////////////

bool fix = false; // determines if the GPS has a lock
unsigned long int fixTimer = 0;


////////////////////////////// Radio related variables ///////////////////////////////////

bool commandRelease = false;
String groundCommand;
String xbeeID = "PPOD";
bool xbeeAlternation = false; // this will allow the xbee to alternate between sending data to MOC SOC and data via MOC SOC to the ground
unsigned long int xbeeTimer = 0;
int xbeeRate = 10000; // 10000 millis = 10 seconds

String xbeeMessage; // This saves all xbee transmissions and appends them to the data string


//////////////////////////// SMART related variables ////////////////////////////////////

bool smartRelease = false; // Releases the smart unit to deploy cubes
float cutTime = 70*60000.0; // x (minutes) * 60000;
float cutAltitude = 50000; // 50,000 ft ASL cut!!
int smartReleasePosition = 180; // the angle in degrees for smart release. Will vary based on smart unit
int smartInitialPosition = 0; // the angle in degrees for smart initial position. Will vary based on smart unit
bool smartReleaseTransmission = true; // This ensures only one message is relayed to the ground after release has occurred.
String commandMessage; // Appends a message stating the radio deployed the cubes if that happens


///////////////////// SD and Datalogging related variables /////////////////////////////

File datalog;
char filename[] = "SDPPOD00.csv"; //File name template
bool sdActive = false;

String header = "GPS Time, Lat, Lon, Alt (ft), AltEstimation (ft), dallas1(F), dallas2(F), pressure1 (PSI), pressure2 (PSI), release?, time since bootup (s), recent xbee message";
String data;
String groundData;
unsigned long int dataTimer = 0;
int dataRate = 1000; // 1000 millis = 1 second

String exclamation = "!"; // this needs to be at the end of every 
String mocSocID = "SOC?";
String mocSocRelay = "$P$";

int analogResolutionBits = 12;
int analogResolutionVals = pow(2,analogResolutionBits);
float pressureBoundary1;
float pressureBoundary2;
float pressureBoundary3;
float pressureOnePSI;
float pressureTwoPSI;
float altitudeFt;

float dallasOneF;
float dallasTwoF;

float magnetometer[3]; // {x, y, z}
float accelerometer[3]; // {x, y, z}
float gyroscope[3]; // {x, y, z}


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Sensor Decleration ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

Servo smart;
LSM9DS1 imu;
RelayXBee xbee = RelayXBee(&xbeeSerial, xbeeID);
UbloxGPS ublox(&ubloxSerial);
OneWire oneWireOne(dallasOnePin); //For Dallas sensor
OneWire oneWireTwo(dallasTwoPin); //For Dallas sensor
DallasTemperature dallasOne(&oneWireOne); //For Dallas sensor
DallasTemperature dallasTwo(&oneWireTwo); //For Dallas sensor


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Function Decleration //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void sdSetup();
void xbeeSetup();
void ubloxSetup();
void imuSetup();
void pressureToAltitudeSetup();
void rfd900Setup();
void ppodSetup();
void magnetometerBootup();

void logData(String Data);
void updateXbee();
void updateIMU();
void updatePressure();
void pressureToAltitude();
void updateDallas();
void updateData();
void blinkSequence();
void Blink(int led, bool ledVal);


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// SETUP /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin(serialBAUD); //define baud rate in variable decleration above
  
  pinMode(fixLED,OUTPUT); 
  pinMode(xbeeLED,OUTPUT);
  pinMode(statusLED,OUTPUT);
  pinMode(uniqueLED,OUTPUT);
  
  sdSetup();
  xbeeSetup();
  ubloxSetup();
  pressureToAltitudeSetup();

  if(magnetBootup) magnetometerBootup();
  if(ppod) ppodSetup();
  if(rfd900) rfd900Setup();
  
  logData(header);
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LOOP ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  
  updateData();
  
}

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Setup Functions  //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void sdSetup(){
  pinMode(chipSelect,OUTPUT);
  if(!SD.begin(chipSelect)){
    Serial.println("Card failed, or not present");
    //bad blink sequence
  }
  else {
    Serial.println("Card initialized.\nCreating File...");
    for (byte i = 0; i < 100; i++) {
      filename[6] = '0' + i/10; 
      filename[7] = '0' + i%10; //filename must be x characters for this to work ("filename[x] = ")
      if (!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        sdActive = true;
        Serial.println("Logging to: " + String(filename));
        break;}
    }
    if (!sdActive) Serial.println("No available file names; clear SD card to enable logging");
  }
}


void xbeeSetup(){
  xbeeSerial.begin(XBEE_BAUD);
  xbee.init('A'); // Need to make sure xbees on both ends have the same identifier. "AAAA"
  xbee.enterATmode();
  xbee.atCommand("ATDL0");
  xbee.atCommand("ATMY1");
  xbee.exitATmode();
  Serial.println("Xbee initialized");
}


void ubloxSetup(){
  ubloxSerial.begin(UBLOX_BAUD);
  ublox.init();
  delay(10);
  Serial.println("Ublox initialized");
  byte i = 0;
  while (i<50) {
    i++;
    if (ublox.setAirborne()) {
      Serial.println("Air mode successfully set.");
      break;}
    if (i==50) Serial.println("Failed to set to air mode.");
  }
}


void imuSetup(){
  Wire.begin();
  if(!imu.begin()) Serial.println("Failed to communicate with LSM9DS1.");
}


void pressureToAltitudeSetup()
{
  float h1 = 36152.0;
  float h2 = 82345.0;
  float T1 = 59-.00356*h1;
  float T2 = -70;
  float T3 = -205.05 + .00164*h2;
  pressureBoundary1 = (2116 * pow(((T1+459.7)/518.6),5.256));
  pressureBoundary2 = (473.1*exp(1.73-.000048*h2)); // does exp function work??
  pressureBoundary3 = (51.97*pow(((T3 + 459.7)/389.98),-11.388));
}


void rfd900Setup(){
  ;// code
}


void ppodSetup(){
  smart.attach(smartPin);
  smart.write(smartInitialPosition);
}


void magnetometerBootup(){
  if( magnetometer[0] > 4.0 || magnetometer[1] > 4.0 || magnetometer[2] > 4.0 ) bootup = true;
  // bootup!; 
  // code
}


void logData(String Data){
  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(Data);
  datalog.close();
  Serial.println(Data);
}


void updateXbee(){ // This is disgusting
  
  // Key: (keep this up to date)
  //
  // T      reports back time remaining until cut.
  // +##    adds time to cut timer 
  // -##    subtracts time to cut timer
  // A      reports remaining altitude until cut.
  // A+##   adds altitude (m) to cut altitude
  // A-##   subtracts altitude (m) to cut altitude
  // C      releases cubes from PPOD
  // D      sends data string
  // M      Polo! just a ping command

  if(millis() - xbeeTimer > xbeeRate){
    if(xbeeAlternation) xbeeAlternation = false;
    else{ xbeeAlternation = true; }// This is so each xbeeRate cycle can alternate between sending data and
    
    xbeeTimer = millis();
    if(xbeeAlternation) xbeeSerial.print(mocSocID + "R" + String(smartRelease) + exclamation); // This would be in the format "SOC?R0!" Where the 0 is a boolean value -- 0 or 1 -- where 1 indicates cube deployment
    
    else{ xbeeSerial.print(mocSocID + mocSocRelay + groundData + exclamation); } // This sends data through MOC-SOC to the ground
  }
  
  if (xbeeSerial.available() > 0) {
    //groundCommand = xbeeSerial.readString(); //for troubleshooting only
    groundCommand = xbee.receive();

    if(groundCommand.startsWith("T"))
    {
      xbeeMessage = (mocSocID + mocSocRelay + String(cutTime-millis()) + exclamation);
    }
    else if(groundCommand.startsWith("+"))
    {
      groundCommand.remove(0,1);
      float timeAdded = groundCommand.toFloat();
      cutTime = cutTime + timeAdded;
      xbeeMessage = (mocSocID + mocSocRelay + "New cut time: " + String(cutTime) + exclamation);
    }
    else if(groundCommand.startsWith("-"))
    {
      groundCommand.remove(0,1);
      float timeSubtracted = groundCommand.toFloat();
      cutTime = cutTime - timeSubtracted;
      xbeeMessage = (mocSocID + mocSocRelay + "New cut time: " + String(cutTime) + exclamation);
    }
    else if(groundCommand.startsWith("A"))
    {
      xbeeMessage = (mocSocID + mocSocRelay + "Altitude calculated from pressure: " + String(altitudeFt) + exclamation);
    }
    else if(groundCommand.startsWith("A+"))
    {
      groundCommand.remove(0,1);
      float altitudeAdded = groundCommand.toFloat();
      cutAltitude = cutAltitude + altitudeAdded;
      xbeeMessage = (mocSocID + mocSocRelay + "New cut altitude: " + String(cutAltitude) + exclamation);
    }
    else if(groundCommand.startsWith("A-"))
    {
      groundCommand.remove(0,1);
      float altitudeSubtracted = groundCommand.toFloat();
      cutAltitude = cutAltitude - altitudeSubtracted;
      xbeeMessage = (mocSocID + mocSocRelay + "New cut altitude: " + String(cutAltitude) + exclamation);
    }
    else if(groundCommand.startsWith("C"))
    {
      if(smartRelease) xbee.send(mocSocID + mocSocRelay + "Cubes deployed prior to command" + exclamation);
      else{ smartRelease = true;
            commandRelease = true;
      }
    }
    else if(groundCommand.startsWith("D"))
    {
      xbeeMessage = (mocSocID + mocSocRelay + data + exclamation);
    }
    else if(groundCommand.startsWith("M"))
    {
      xbeeMessage = (mocSocID + mocSocRelay + "Polo" + exclamation);
    }
    else{
      xbeeMessage = (mocSocID + mocSocRelay + "Error - command not recognized: " + groundCommand + exclamation);
    }
    xbee.send(xbeeMessage);
  }
}


void updateIMU(){
  if( imu.gyroAvailable() ) imu.readGyro();
  if( imu.accelAvailable() ) imu.readAccel();
  if( imu.magAvailable() ) imu.readMag();

  magnetometer[0] = imu.calcMag(imu.mx);
  magnetometer[1] = imu.calcMag(imu.my);
  magnetometer[2] = imu.calcMag(imu.mz);
  accelerometer[0] = imu.calcAccel(imu.ax);
  accelerometer[1] = imu.calcAccel(imu.ay);
  accelerometer[2] = imu.calcAccel(imu.az);
  gyroscope[0] = imu.calcGyro(imu.gx);
  gyroscope[1] = imu.calcGyro(imu.gy);
  gyroscope[2] = imu.calcGyro(imu.gz);
}


void updatePressure() { // Output units: psi -- far from efficient, but works for our purpose
  analogReadResolution(analogResolutionBits);
  float rawPressure = analogRead(pressureOnePin);
  float pressureVoltage = rawPressure*(5.0/analogResolutionVals);
  float pressure = ((pressureVoltage - 0.5)*(15.0/4.0));
  pressureOnePSI = pressure;

  rawPressure = analogRead(pressureTwoPin);
  pressureVoltage = rawPressure*(5.0/analogResolutionVals);
  pressure = ((pressureVoltage - 0.5)*(15.0/4.0));
  pressureTwoPSI = pressure;
}


void pressureToAltitude(){
  float pressurePSF = (pressureOnePSI*144);
  
  float altFt = -1.0;
  if (pressurePSF > pressureBoundary1)// altitude is less than 36,152 ft ASL
  {
    altFt = (459.7+59-518.6*pow((pressurePSF/2116),(1/5.256)))/.00356;
  }
  if (pressurePSF <= pressureBoundary1 && pressurePSF > pressureBoundary2) // altitude is between 36,152 and 82,345 ft ASL
  {
    altFt = (1.73-log(pressurePSF/473.1))/.000048;
  }
  if (pressurePSF <= pressureBoundary2)// altitude is greater than 82,345 ft ASL
  {
    altFt = (459.7-205.5-389.98*pow((pressurePSF/51.97),(1/-11.388)))/-.00164;
  }
  altitudeFt = altFt;
}


void updateDallas(){ // This looks different from pressure retreival because it is digital, not analog
  dallasOne.requestTemperatures();
  dallasTwo.requestTemperatures();
  dallasOneF = dallasOne.getTempFByIndex(0);
  dallasTwoF = dallasTwo.getTempFByIndex(0);
}


void updateData(){
  ublox.update();
  blinkSequence();
  updateXbee();
  
  if(millis() - dataTimer > dataRate){
    dataTimer = millis();
    
    updateDallas();
    updatePressure();
    //updateIMU();
    updateSmart();
    
    data = String(ublox.getHour()-5) + "," + String(ublox.getMinute()) + "," + String(ublox.getSecond()) + ", "
           + String(ublox.getLat(), 4) + ", " + String(ublox.getLon(), 4) + ", " + String(ublox.getAlt_feet(), 4)
           +  ", " + String(altitudeFt) + "," + String(dallasOneF) + ", " + String(dallasTwoF) + ", "
           + String(pressureOnePSI) + ", " + String(pressureTwoPSI) + ", " + String(smartRelease) + ", " + String(millis()) + ", " + xbeeMessage;

    groundData = "GPS altitude (ft):" + String(ublox.getAlt_feet(),4) + ", Estimated Altitude(ft):" + String(altitudeFt)
                  + ", millis(): " + String(millis()) + ", cutTime: " + String(cutTime);
                 
    if(ublox.getFixAge() > 2000) fix = false;
    else fix = true;

    logData(data);
    xbeeMessage = "";
  }         
}


void updateSmart()
{
  if(altitudeFt > cutAltitude || millis() > cutTime)
  {
    smartRelease = true;
  }
  
  if(smartRelease){
    smart.write(smartReleasePosition);
    
    if(commandRelease){
      commandMessage = " via command";
      commandRelease = false;
    }
    
    if(smartReleaseTransmission){
      xbeeMessage = (mocSocID + mocSocRelay + "cubes deployed" + commandMessage + "! Altitude(ft): " + String(altitudeFt) + "; Time = " + String(millis()) + exclamation);
      smartReleaseTransmission = false;
      xbee.send(xbeeMessage);
    }
  }
}


void blinkSequence()
{
  if(fix){
    if(millis()-fixTimer > 1000){
      fixTimer = millis();
      //Blink(fixLED,fixLED);
    }
  }
  else{
    if(millis()-fixTimer > 200){
      fixTimer = millis();
      //Blink(fixLED,fixLEDval);
    }
  }
}


void Blink(int led, bool ledVal)
{
  ledVal = !ledVal;
  digitalWrite(led,ledVal);
}
