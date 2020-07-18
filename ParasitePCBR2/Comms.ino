#include <RelayXBee.h>

#define xbeeSerial Serial5        // Serial communication lines for the xbee radio -- PCB pins: Serial3
//#define rfd900Serial serialX    //this line should all be commented if you aren't using an RFD900

RelayXBee xbee = RelayXBee(&xbeeSerial, xbeeID);

void xbeeSetup(){
  
  xbeeSerial.begin(XBEE_BAUD);
  xbee.init('A'); // Need to make sure xbees on both ends have the same identifier. "AAAA"
  xbee.enterATmode();
  xbee.atCommand("ATDL0");
  xbee.atCommand("ATMY1");
  xbee.exitATmode();
  Serial.println("Xbee initialized");
  
}

void rfd900Setup(){
  digitalWrite(xbeeLED,HIGH);
  Serial.print("Initializing Long Range Radio... ");
  // code
  Serial.println("Radio Initialized");
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
    xbee.send(groundData); 
    xbeeTimer = millis();
  }
  
  if (xbeeSerial.available() > 0) {
    groundCommand = xbee.receive();

    if(groundCommand.startsWith("TIME"))
    {
      xbeeMessage = String(cutTime-millis());
    }
    else if(groundCommand.startsWith("+"))
    {
      groundCommand.remove(0,1);
      float timeAdded = groundCommand.toFloat();
      cutTime = cutTime + timeAdded;
      xbeeMessage = "New cut time: " + String(cutTime);
    }
    else if(groundCommand.startsWith("-"))
    {
      groundCommand.remove(0,1);
      float timeSubtracted = groundCommand.toFloat();
      cutTime = cutTime - timeSubtracted;
      xbeeMessage = "New cut time: " + String(cutTime);
    }
    else if(groundCommand.startsWith("ALT"))
    {
      xbeeMessage = "Altitude calculated from pressure: " + String(altitudeFt);
    }
    else if(groundCommand.startsWith("A+"))
    {
      groundCommand.remove(0,1);
      float altitudeAdded = groundCommand.toFloat();
      cutAltitude = cutAltitude + altitudeAdded;
      xbeeMessage = "New cut altitude: " + String(cutAltitude);
    }
    else if(groundCommand.startsWith("A-"))
    {
      groundCommand.remove(0,1);
      float altitudeSubtracted = groundCommand.toFloat();
      cutAltitude = cutAltitude - altitudeSubtracted;
      xbeeMessage = "New cut altitude: " + String(cutAltitude);
    }
    else if(groundCommand.startsWith("CUT"))
    {
      if(smartRelease) xbee.send("Cubes deployed prior to command");
      else{ smartRelease = true;
            commandRelease = true;
      }
    }
    else if(groundCommand.startsWith("DATA"))
    {
      xbeeMessage = data;
    }
    else if(groundCommand.startsWith("MARCO"))
    {
      xbeeMessage = "POLO";
    }
    else{
      xbeeMessage = "Error - command not recognized: " + groundCommand;
    }
    xbee.send(xbeeMessage);
  }
}
