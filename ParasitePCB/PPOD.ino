#include <Servo.h> 

#define smartPin 2               // The data pin on the servo motor. Must have PWM capabilities  -- PCB pin: 2

Servo smart;

void ppodSetup(){
  smart.attach(smartPin);
  smart.write(smartInitialPosition);
}

void updateSmart(){
  if(altitudeFt > cutAltitude || millis() > cutTime){ smartRelease = true; }
  
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
