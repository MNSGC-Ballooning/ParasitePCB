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
#define thermIntPin A16
#define thermExtPin A17
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
  //delay(1000);
  //oled.clear(PAGE);
}

void msSetup() {
  baro.begin();
}

void updateMS() {
  msTemperature = baro.readTemperature();
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
  data = String(ublox.getMonth()) + "/" + String(ublox.getDay()) + "/" + String(ublox.getYear()) + "\t" + 
          String(ublox.getHour()-5) + ":" + String(ublox.getMinute()) + ":" + String(ublox.getSecond()) + "\t "
           + String(ublox.getLat(), 4) + "\t" + String(ublox.getLon(), 4) + "\t " + String(ublox.getAlt_feet(), 5)
           +  "\t " + String(altitudeFt) + "\t" + String(thermistorInt) + "\t " + String(thermistorExt) + "\t"
           + "\t" + String(msTemperature) + "\t " + String(pressureOnePSI) + "\t" + String(msPressure) + "\t" + String(millis()) + "\t" + xbeeMessage;

 groundData = String(ublox.getMonth()) + "/" + String(ublox.getDay()) + "/" + String(ublox.getYear()) + "," +
            String(ublox.getHour()-5) + ":" + String(ublox.getMinute()) + ":" + String(ublox.getSecond()) + ","
           + String(ublox.getLat(), 4) + ", " + String(ublox.getLon(), 4) + ", " + String(ublox.getAlt_feet(), 4)
           +  ", " + String(altitudeFt) + ", " + String(thermistorInt) + ", " + String(thermistorExt) + ", "
           + String(msTemperature) + ", " + String(pressureOnePSI) + ", " + String(msPressure) + "," + String(millis()) + ", " + xbeeMessage;

  updateOled("GPS Alt:\n" + String(ublox.getAlt_feet(),1) + "ft\n\nGuess:\n" + String(altitudeFt) + "ft");
  if(ublox.getFixAge() > 2000) fix = false;
  else fix = true;
  logData(data);
}

void magnetometerBootup(){
  while((abs(magnetometer[0]) < 3.50) || (abs(magnetometer[1]) < 3.50) || (abs(magnetometer[2]) < 3.50))
  {
    digitalWrite(fixLED,HIGH);
    updateIMU();
  }
  digitalWrite(fixLED,LOW);
}
