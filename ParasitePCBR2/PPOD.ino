#include <Servo.h> 

#define smartPin 2               // The data pin on the servo motor. Must have PWM capabilities  -- PCB pin: 2

Servo smart;

unsigned long int testTimer;

void ppodSetup(){
  
  digitalWrite(ppodLED,HIGH);
  digitalWrite(xbeeLED,HIGH);
  digitalWrite(fixLED,HIGH);
  digitalWrite(sdLED,HIGH);
  Serial.print("Initializing PPOD... ");
  smart.attach(smartPin);
  smart.write(smartInitialPosition);
  Serial.println("PPOD initialized");
  delay(10);

  while(setAltVal==1){
    setAltVal = digitalRead(setAltSwitch);
    altVal = digitalRead(altSwitch);
    if(altVal==0){
      cutAltitude+=10000;
      if(cutAltitude>=120000){
        cutAltitude=10000;
      }
     }
     if(cutAltitude == 110000){
        updateOled("Set 30 second servo test");
        delay(100);
      }
      else{
        updateOled("Set PPOD\nAlt:\n\n" + String(int(cutAltitude)) + "ft");
        delay(100);
      }
   }
   updateOled("Alt Set!\n\n" + String(int(cutAltitude)) + "ft");
   if(cutAltitude==110000) updateOled("Alt Set!\n\n30 Second Test");
   delay(2000);
   testTimer = millis();
   digitalWrite(ppodLED,LOW);
   digitalWrite(xbeeLED,LOW);
   digitalWrite(fixLED,LOW);
   digitalWrite(sdLED,LOW);
  }

void updateSmart(){
  if (cutAltitude == 110000 && (millis()-testTimer>30000)) { smartRelease = true; }
  
  if(altitudeFtGPS > cutAltitude || millis() > cutTime){ smartRelease = true; }
  
  if(smartRelease){
    smart.write(smartReleasePosition);
    
    if(commandRelease){
      commandMessage = " via command";
      commandRelease = false;
    }
    
    if(smartReleaseTransmission){
      xbeeMessage = "cubes deployed" + commandMessage + "! Altitude(ft): " + String(altitudeFt) + "; Time = " + String(millis());
      smartReleaseTransmission = false;
      xbee.send(xbeeMessage);
    }
  }
  if(smartRelease == true)digitalWrite(ppodLED,HIGH);
    
  }
