#include <RelayXBee.h>

#define xbeeSerial Serial5        // Serial communication lines for the xbee radio -- PCB pins: Serial3
//#define rfd900Serial serialX    //this line should all be commented if you aren't using an RFD900

String exclamation = "!"; // this needs to be at the end of every 
String mocSocID = "SOC?";
String mocSocRelay = "$P$";
bool commandRelease = false;
String groundCommand;
String xbeeID = "PPOD";
bool xbeeAlternation = false; // this will allow the xbee to alternate between sending data to MOC SOC and data via MOC SOC to the ground
unsigned long int xbeeTimer = 0;
int xbeeRate = 1000; // 10000 millis = 10 seconds
String xbeeMessage; // This saves all xbee transmissions and appends them to the data string


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
  ;// code
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
