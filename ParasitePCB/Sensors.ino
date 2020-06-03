#include <UbloxGPS.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define dallasOnePin 29           // Data pin for the first dallas temperature sensor -- PCB pin:
#define dallasTwoPin 30           // Data pin for the second dallas temperature sensor -- PCB pin: 
#define pressureOnePin A1         // Data pin for the first honeywell pressure sensor -- PCB pin:
#define pressureTwoPin A2         // Data pin for the second honeywell pressure sensor -- PCB pin:
#define ubloxSerial Serial4       // Serial communication lines for the ublox GPS -- PCB pins: Serial5

LSM9DS1 imu;
UbloxGPS ublox(&ubloxSerial);
OneWire oneWireOne(dallasOnePin); //For Dallas sensor
OneWire oneWireTwo(dallasTwoPin); //For Dallas sensor
DallasTemperature dallasOne(&oneWireOne); //For Dallas sensor
DallasTemperature dallasTwo(&oneWireTwo); //For Dallas sensor

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

void updateDallas(){ // This looks different from pressure retreival because it is digital, not analog
  dallasOne.requestTemperatures();
  dallasTwo.requestTemperatures();
  dallasOneF = dallasOne.getTempFByIndex(0);
  dallasTwoF = dallasTwo.getTempFByIndex(0);
}

void updateUblox(){
  ublox.update();
}

void updateDataStrings(){
  data = String(ublox.getHour()-5) + "\t" + String(ublox.getMinute()) + "\t" + String(ublox.getSecond()) + "\t "
           + String(ublox.getLat(), 4) + "\t" + String(ublox.getLon(), 4) + "\t " + String(ublox.getAlt_feet(), 4)
           +  "\t " + String(altitudeFt) + "\t" + String(dallasOneF) + "\t " + String(dallasTwoF) + "\t"
           + String(pressureOnePSI) + "\t" + String(pressureTwoPSI) + "\t " + String(smartRelease) + "\t" + String(millis()) + "\t" + xbeeMessage;

 groundData = String(ublox.getHour()-5) + ", " + String(ublox.getMinute()) + ", " + String(ublox.getSecond()) + ", "
           + String(ublox.getLat(), 4) + ", " + String(ublox.getLon(), 4) + ", " + String(ublox.getAlt_feet(), 4)
           +  ", " + String(altitudeFt) + ", " + String(dallasOneF) + ", " + String(dallasTwoF) + ", "
           + String(pressureOnePSI) + ", " + String(pressureTwoPSI) + ", " + String(smartRelease) + ", " + String(millis()) + ", " + xbeeMessage;
                 
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
}
