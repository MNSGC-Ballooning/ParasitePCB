#include <RelayXBee.h>

#define xbeeSerial Serial5        // Serial communication lines for the xbee radio -- PCB pins: Serial3
#define rfd900Serial Serial1   //this line should all be commented if you aren't using an RFD900

RelayXBee xbee = RelayXBee(&xbeeSerial, xbeeID);

void xbeeSetup(){
  updateOled("Xbee Radio\nInit...");
  char xbeeChannel = 'A';
  xbeeSerial.begin(XBEE_BAUD);
  xbee.init(xbeeChannel); // Need to make sure xbees on both ends have the same identifier. "AAAA"
  xbee.enterATmode();
  if(rfd900==0){
    xbee.atCommand("ATDL1");
    xbee.atCommand("ATMY0");
  }
  else{
    xbee.atCommand("ATDL0");
    xbee.atCommand("ATMY1");
  }
  xbee.exitATmode();
  Serial.println("Xbee initialized on channel: " + String(xbeeChannel) + "; ID: " + xbeeID);
  updateOled("Xbee\nChannel: " + String(xbeeChannel) + "\nID: " + xbeeID);
  delay(2000);
}

void rfd900Setup(){
  digitalWrite(xbeeLED,HIGH);
  Serial.print("Initializing Long Range Radio... ");
  updateOled("Setting up RFD900 on Serial1");
  rfd900Serial.begin(57600); // RFD900 Baud
  delay(2000);
  
  Serial.println("Radio Initialized");
  digitalWrite(xbeeLED,LOW);
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

  xbeeMessage = "";

  if(millis() - xbeeTimer > xbeeRate){ 
    
    xbeeTimer = millis();
    
    if(rfd900==0){rfd900Serial.println(xbeeID + "," + groundData + "!");
    }
    else xbeeSerial.print(xbeeID + "," + groundData + "!"); 

    digitalWrite(xbeeLED,HIGH);
    delay(80);
    digitalWrite(xbeeLED,LOW);
  }
  
  if (xbeeSerial.available() > 20 && rfd900==0) { // RELAY!!
    groundCommand = xbeeSerial.readString();
    xbeeSerial.flush();
    rfd900Serial.println(groundCommand);
    }
  if (rfd900==0 && rfd900Serial.available() > 0){
    groundCommand = rfd900Serial.readStringUntil("\r\n");
    Serial.println(groundCommand);
    if(groundCommand.startsWith(xbeeID)){
      groundCommand.remove(0,xbeeID.length()+1);
      rfd900Serial.println(xbeeID + ", " + interpretMessage(groundCommand) + "!");
    }
    else{
      xbeeSerial.print(groundCommand);
    }
  }
}

String interpretMessage( String myCommand ){
    
    if(myCommand.startsWith("TIME"))
    {
      xbeeMessage = String(cutTime-millis());
    }
    else if(myCommand.startsWith("+"))
    {
      myCommand.remove(0,1);
      float timeAdded = myCommand.toFloat();
      cutTime = cutTime + timeAdded;
      xbeeMessage = "New cut time: " + String(cutTime);
    }
    else if(myCommand.startsWith("-"))
    {
      myCommand.remove(0,1);
      float timeSubtracted = myCommand.toFloat();
      cutTime = cutTime - timeSubtracted;
      xbeeMessage = "New cut time: " + String(cutTime);
    }
    else if(myCommand.startsWith("ALT"))
    {
      xbeeMessage = "Altitude calculated from pressure: " + String(altitudeFt);
    }
    else if(myCommand.startsWith("A+"))
    {
      myCommand.remove(0,1);
      float altitudeAdded = myCommand.toFloat();
      cutAltitude = cutAltitude + altitudeAdded;
      xbeeMessage = "New cut altitude: " + String(cutAltitude);
    }
    else if(myCommand.startsWith("A-"))
    {
      myCommand.remove(0,1);
      float altitudeSubtracted = myCommand.toFloat();
      cutAltitude = cutAltitude - altitudeSubtracted;
      xbeeMessage = "New cut altitude: " + String(cutAltitude);
    }
    else if(myCommand.startsWith("CUT"))
    {
      if(smartRelease) xbee.send("Cubes deployed prior to command");
      else{ smartRelease = true;
            commandRelease = true;
      }
    }
    else if(myCommand.startsWith("DATA"))
    {
      xbeeMessage = data;
    }
    else if(myCommand.startsWith("MARCO"))
    {
      xbeeMessage = "POLO";
    }
    else if(myCommand.startsWith("FREQ="))
    {
      myCommand.remove(0,5);
      xbeeMessage = "New Send Rate: " + groundCommand;
      xbeeRate = groundCommand.toInt();
    }
    else{
      xbeeMessage = "Error - command not recognized: " + groundCommand;
    }
    if(xbeeMessage!="") return xbeeMessage;
  }
