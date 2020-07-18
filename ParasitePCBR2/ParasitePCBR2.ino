#define fixLED 37                 // LED to indicate GPS fix
#define ppodLED 38
#define xbeeLED 39                // LED to indicate xbee communication
#define sdLED 20
#define serialBAUD 9600           // when using arduino serial monitor, make sure baud rate is set to this same value

#define ppodSwitchPin 23
#define magnetSwitchPin 22
#define commsSwitchPin 21
#define setAltSwitch 17
#define altSwitch 16

int rfd900 = 1; // set true if you want this thing to operate with a rfd900 on serialX (declare above)
int ppod = 1; // set true if you want this thing to operate as ppod flight computer
int magnetBootup = 1; // set true if you want to activate flight by waving a magnet over the IMU
int setAltVal = 1;
int altVal = 1;

float cutTime = 70*60000.0; // x (minutes) * 60000;
float cutAltitude = 10000; // 50,000 ft ASL cut!!
bool smartRelease = false; // Releases the smart unit to deploy cubes
int smartReleasePosition = 170; // the angle in degrees for smart release. Will vary based on smart unit
int smartInitialPosition = 0; // the angle in degrees for smart initial position. Will vary based on smart unit
bool smartReleaseTransmission = true; // This ensures only one message is relayed to the ground after release has occurred.
String commandMessage; // Appends a message stating the radio deployed the cubes if that happens

String header = "Hour \t min \t sec \t Lat \t Lon \t Alt(ft) \t AltEst(ft) \t intT(F) \t extT(F) \t msTemp (F) \t msPressure(Pa) \t time since bootup (s) \t recent xbee message";
unsigned long int dataTimer = 0;
int dataRate = 1000; // 1000 millis = 1 second
int analogResolutionBits = 14;
int analogResolutionVals = pow(2,analogResolutionBits);
float pressureBoundary1;
float pressureBoundary2;
float pressureBoundary3;
float pressureOnePSI;
float msPressure;
float msTemperature;
float altitudeFt;
unsigned long int fixTimer = 0;
bool fix = false; // determines if the GPS has a lock

///////////////////// Sensor Global Variables /////////////////////////////

float dallasOneF;
float thermistorInt;
float thermistorExt;
float magnetometer[3]; // {x, y, z}
float accelerometer[3]; // {x, y, z}
float gyroscope[3]; // {x, y, z}

String data;
String groundData;

String exclamation = "!"; // this needs to be at the end of every 
bool commandRelease = false;
String groundCommand;
String xbeeID = "PPOD";
bool xbeeAlternation = false; // this will allow the xbee to alternate between sending data to MOC SOC and data via MOC SOC to the ground
unsigned long int xbeeTimer = 0;
int xbeeRate = 5000; // 10000 millis = 10 seconds
String xbeeMessage; // This saves all xbee transmissions and appends them to the data string

void setup() {
  
  Serial.begin(serialBAUD); //define baud rate in variable decleration above
  Serial.println("Serial online");
  pinMode(fixLED,OUTPUT); 
  pinMode(xbeeLED,OUTPUT);
  pinMode(13,OUTPUT);

  checkSwitches();

  Serial.print("starting OLED setup... ");
  oledSetup();
  Serial.println("OLED setup complete");
  
  Serial.print("starting IMU setup... ");
  imuSetup();
  updateIMU();
  Serial.println("IMU setup complete");

  Serial.print("starting Altimeter setup... ");
  oledSetup();
  msSetup();
  Serial.println("Altimeter setup complete");

  Serial.print("starting SD setup... ");
  sdSetup();
  Serial.println("SD setup complete");

  Serial.print("starting xbee setup... ");
  xbeeSetup();
  Serial.println("xbee setup complete");

  Serial.print("starting ublox setup... ");
  ubloxSetup();
  Serial.println("ublox setup complete");
  
  pressureToAltitudeSetup();
  if(ppod==0) ppodSetup();
  if(rfd900==0) rfd900Setup();
  logData(header);
  if(magnetBootup==0) magnetometerBootup();
  buzzerSequence();  

  digitalWrite(xbeeLED, HIGH);
}

void loop() {
  updateData(); 
}

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Functions  ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

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

void pressureToAltitude(){
  //float pressurePSF = (pressureOnePSI*144);
  float pressurePSF = (msPressure*144);
  
  float altFt = -100.0;
  //UNCOMMENT WHEN RELIABLE PRESSURE SENSORS ARE ON BOARD
  if (pressurePSF > pressureBoundary1)// altitude is less than 36,152 ft ASL
  {
    altFt = (459.7+59-518.6*pow((pressurePSF/2116),(1/5.256)))/.00356;
  }
  else if (pressurePSF <= pressureBoundary1 && pressurePSF > pressureBoundary2) // altitude is between 36,152 and 82,345 ft ASL
  {
    altFt = (1.73-log(pressurePSF/473.1))/.000048;
  }
  else if (pressurePSF <= pressureBoundary2)// altitude is greater than 82,345 ft ASL
  {
    altFt = (459.7-205.5-389.98*pow((pressurePSF/51.97),(1/-11.388)))/-.00164;
  }
  else{altFt = -1.0;}
  
  altitudeFt = altFt;
}

void updateData(){
  updateUblox();
  blinkSequence();
  updateXbee();
  
  if(millis() - dataTimer > dataRate){
    dataTimer = millis();
    digitalWrite(xbeeLED,HIGH);
    delay(100);
    digitalWrite(xbeeLED,LOW); // TESTING
    pressureToAltitude();
    updateDallas();
    updateThermistor();
    updateMS();
    updatePressure();
    updateIMU();
    updateSmart();
    updateDataStrings();
  }        
}

void blinkSequence(){
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


void Blink(int led, bool ledVal){
  ledVal = !ledVal;
  digitalWrite(led,ledVal);
}

void buzzerSequence(){
  tone(38,2450);
  delay(100);
  noTone(38);
  delay(10);
  tone(38,2400);
  delay(100);
  noTone(38);
  delay(10);
  tone(38,2450);
  delay(100);
  noTone(38); 
}

void checkSwitches(){
  pinMode(ppodSwitchPin, INPUT_PULLUP);
  pinMode(magnetSwitchPin, INPUT_PULLUP);
  pinMode(commsSwitchPin, INPUT_PULLUP);

  //digitalWrite(fixLED,HIGH);

  ppod = digitalRead(ppodSwitchPin);
  rfd900 = digitalRead(commsSwitchPin);
  magnetBootup = digitalRead(magnetSwitchPin);
}
