#include <Servo.h> 

#define smartPin 2               // The data pin on the servo motor. Must have PWM capabilities  -- PCB pin: 2

Servo smart;

unsigned long int testTimer;

void ppodSetup(){

  digitalWrite(ppodLED,HIGH);
  digitalWrite(xbeeLED,HIGH);
  digitalWrite(ppodLED,HIGH);
  digitalWrite(ppodLED,HIGH);
  Serial.print("Initializing PPOD... ");
  smart.attach(smartPin);
  smart.write(smartInitialPosition);
  Serial.println("PPOD initialized");
  pinMode(setAltSwitch, INPUT_PULLUP);
  pinMode(altSwitch, INPUT_PULLUP);
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
        delay(50);
      }
      else{
        updateOled("Set Alt:\n" + String(int(cutAltitude)) + "ft");
        delay(50);
      }
   }
   testTimer = millis();
  }

void updateSmart(){
  if (cutAltitude == 110000 && (millis()-testTimer>30000)) { smartRelease = true; }
  
  if(altitudeFt > cutAltitude || millis() > cutTime){ smartRelease = true; }
  
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
}
