#include <UbloxGPS.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MS5611.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library

//The library assumes a reset pin is necessary. The Qwiic OLED has RST hard-wired, so pick an arbitrarty IO pin that is not being used
#define PIN_RESET 9  
//The DC_JUMPER is the I2C Address Select jumper. Set to 1 if the jumper is open (Default), or set to 0 if it's closed.
#define DC_JUMPER 1 

#define dallasOnePin 28           // Data pin for the first dallas temperature sensor -- PCB pin:
#define pressureOnePin A13         // Data pin for the first honeywell pressure sensor -- PCB pin:
#define thermIntPin A17
#define thermExtPin A16
#define ubloxSerial Serial3       // Serial communication lines for the ublox GPS -- PCB pins: Serial5

MS5611 baro;
MicroOLED oled(PIN_RESET, DC_JUMPER);    // I2C declaration
LSM9DS1 imu;
UbloxGPS ublox(&ubloxSerial);
OneWire oneWireOne(dallasOnePin); //For Dallas sensor
DallasTemperature dallasOne(&oneWireOne); //For Dallas sensor

/////////////// Thermistor constants //////////////////////

float adcMax = pow(2,analogResolutionBits)-1.0; // The maximum adc value given to the thermistor
float A = 0.001125308852122;
float B = 0.000234711863267;
float C = 0.000000085663516; // A, B, and C are constants used for a 10k resistor and 10k thermistor for the steinhart-hart equation
float R1 = 10000; // 10k Ω resistor
float Tinv;
float adcVal;
float logR;
float T; // these three variables are used for the calculation from adc value to temperature
float currentTempC; // The current temperature in Celcius
float currentTempF; // The current temperature in Fahrenheit

void ubloxSetup(){
  ubloxSerial.begin(UBLOX_BAUD);
  ublox.init();
  
  byte i = 0;
  while (i<50) {
    i++;
    if (ublox.setAirborne()) {
      Serial.println("Air mode successfully set.");
      break;}
    if (i==50){
      Serial.println("Failed to set to air mode.");
      updateOled("Failed to set GPS Air Mode");
      delay(5000);
    }
  }
  updateOled("GPS init\ncomplete!");
  delay(1000);
}

void imuSetup(){
  Wire.begin();
  if(!imu.begin()){
    Serial.println("Failed to communicate with LSM9DS1.");
    updateOled("IMU\nOffline.");
    delay(5000);
  }
  else{
    updateOled("IMU init\ncomplete!");
    delay(1000);
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

void logIMUdata(){
  IMUdata = String(magnetometer[0]) + ", " + String(magnetometer[1]) + ", " + String(magnetometer[2]) + ", " +
            String(accelerometer[0]) + ", " + String(accelerometer[1]) + ", " + String(accelerometer[2]) + ", " +
            String(gyroscope[0]) + ", " + String(gyroscope[1]) + ", " + String(gyroscope[2]) + ", " + String(millis());
}

void oledSetup(){
  Wire.begin();
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  //delay(1000);     // Delay 1000 ms
  oled.clear(PAGE); // Clear the buffer.

  randomSeed(analogRead(A0) + analogRead(A1));

  updateOled("Initializing...");
}

void updateOled(String disp){
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0, 0);
  oled.println(disp);
  oled.display();
}

void msSetup() {
  //updateOled("initializing\nbaro...");
  while(!baro.begin()){
    updateOled("baro init failed!");
  }
  /*if(!baro.begin()){
    Serial.println("MS5611 Altimeter not active");
    updateOled("digital baro not active");
  }*/
  updateOled("baro init\ncomplete!");
  delay(1000);
}

void updateMS() {
  msTemperature = baro.readTemperature();
  msTemperature = msTemperature*(9.0/5.0) + 32.0;
  msPressure = baro.readPressure(); 
  msPressure = msPressure * 0.000145038;
}

void updatePressure() { // Output units: psi -- far from efficient, but works for our purpose
  analogReadResolution(analogResolutionBits);
  float rawPressure = analogRead(pressureOnePin);
  float pressureVoltage = rawPressure*(3.3/analogResolutionVals);
  float pressure = ((pressureVoltage - 0.33)*(15.0/2.66667));
  pressureOnePSI = pressure;
}

void updateDallas(){ // This looks different from pressure retreival because it is digital, not analog
  dallasOne.requestTemperatures();
  dallasOneF = dallasOne.getTempFByIndex(0);
}

void updateThermistor(){
  analogReadResolution(analogResolutionBits);
  adcVal = analogRead(thermIntPin);
  logR = log(((adcMax/adcVal)-1)*R1);
  Tinv = A+B*logR+C*logR*logR*logR;
  T = 1/Tinv;
  currentTempC = T-273.15; // converting to celcius
  currentTempF = currentTempC*9/5+32;
  thermistorInt = currentTempF;

  adcVal = analogRead(thermExtPin);
  logR = log(((adcMax/adcVal)-1)*R1);
  Tinv = A+B*logR+C*logR*logR*logR;
  T = 1/Tinv;
  currentTempC = T-273.15; // converting to celcius
  currentTempF = currentTempC*9/5+32;
  thermistorExt = currentTempF;
}

void updateUblox(){
  ublox.update();
}

void updateDataStrings(){
  altitudeFtGPS = ublox.getAlt_feet();
  latitudeGPS = ublox.getLat();
  longitudeGPS = ublox.getLon();

 groundData = String(ublox.getMonth()) + "/" + String(ublox.getDay()) + "/" + String(ublox.getYear()) + "," +
            String(ublox.getHour()-5) + ":" + String(ublox.getMinute()) + ":" + String(ublox.getSecond()) + ","
           + String(ublox.getLat(), 4) + ", " + String(ublox.getLon(), 4) + ", " + String(altitudeFtGPS, 4)
           +  ", " + String(altitudeFt) + ", " + String(thermistorInt) + ", " + String(thermistorExt) + ", "
           + String(msTemperature) + ", " + String(pressureOnePSI) + ", " + String(msPressure) + ", " + String(millis()) + ", " + xbeeMessage;

 data = groundData +  ", " + String(magnetometer[0]) + ", " + String(magnetometer[1]) + ", " + String(magnetometer[2]) + ", " +
            String(accelerometer[0]) + ", " + String(accelerometer[1]) + ", " + String(accelerometer[2]) + ", " +
            String(gyroscope[0]) + ", " + String(gyroscope[1]) + ", " + String(gyroscope[2]);
            
  updateOled(String(latitudeGPS) + "\n" + String(longitudeGPS) + "\n" + String(altitudeFtGPS,1) + "ft\nInt:" + String(int(thermistorInt)) + " F\nExt:" + String(int(thermistorExt)) + " F\n" + String(msPressure,2) + " PSI");
  if(ublox.getFixAge() > 2000) fix = false;
  else fix = true;
  logData(data);
}

void magnetometerBootup(){
  updateOled("Wave magnet over IMU to begin logging");
  
  while((abs(magnetometer[0]) < 3.50) || (abs(magnetometer[1]) < 3.50) || (abs(magnetometer[2]) < 3.50))
  {
    ledGlissando();
    updateIMU();
  }
  updateOled("Logging!");
  digitalWrite(fixLED,LOW);
  digitalWrite(ppodLED,LOW);
  digitalWrite(xbeeLED,LOW);
  digitalWrite(sdLED,LOW);
}

void ledGlissando() {
  digitalWrite(fixLED,HIGH);
  delay(50);
  digitalWrite(ppodLED,HIGH);
  delay(50);
  digitalWrite(fixLED,LOW);
  digitalWrite(xbeeLED,HIGH);
  delay(50);
  digitalWrite(ppodLED,LOW);
  digitalWrite(sdLED,HIGH);
  delay(50);
  digitalWrite(xbeeLED,LOW);
  delay(50);
  digitalWrite(sdLED,LOW);
  delay(150);
}
