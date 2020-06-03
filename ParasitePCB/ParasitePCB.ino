////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Pin Decleration ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

#define fixLED 28                 // LED to indicate GPS fix
#define xbeeLED 27                // LED to indicate xbee communication
#define statusLED 37              // LED to flash each data cycle
#define uniqueLED 18             // 
#define serialBAUD 9600           // when using arduino serial monitor, make sure baud rate is set to this same value

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Variable Decleration //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

bool rfd900 = false; // set true if you want this thing to operate with a rfd900 on serialX (declare above)
bool ppod = true; // set true if you want this thing to operate as ppod flight computer
bool magnetBootup = true; // set true if you want to activate flight by waving a magnet over the IMU

//////////////////////////// SMART related variables ////////////////////////////////////

float cutTime = 70*60000.0; // x (minutes) * 60000;
float cutAltitude = 50000; // 50,000 ft ASL cut!!
bool smartRelease = false; // Releases the smart unit to deploy cubes
int smartReleasePosition = 0; // the angle in degrees for smart release. Will vary based on smart unit
int smartInitialPosition = 170; // the angle in degrees for smart initial position. Will vary based on smart unit
bool smartReleaseTransmission = true; // This ensures only one message is relayed to the ground after release has occurred.
String commandMessage; // Appends a message stating the radio deployed the cubes if that happens

///////////////////// SD and Datalogging related variables /////////////////////////////

String header = "Hour \t minute \t second \t Lat \t Lon \t Alt (ft) \t AltEstimation (ft) \t dallas1(F) \t dallas2(F) \t pressure1 (PSI) \t pressure2 (PSI) \t ax (g's) \t ay \t az \t release? \t time since bootup (s) \t recent xbee message";
unsigned long int dataTimer = 0;
int dataRate = 1000; // 1000 millis = 1 second
int analogResolutionBits = 12;
int analogResolutionVals = pow(2,analogResolutionBits);
float pressureBoundary1;
float pressureBoundary2;
float pressureBoundary3;
float pressureOnePSI;
float pressureTwoPSI;
float altitudeFt;
unsigned long int fixTimer = 0;
bool fix = false; // determines if the GPS has a lock

///////////////////// Sensor Global Variables /////////////////////////////

float dallasOneF;
float dallasTwoF;
float magnetometer[3]; // {x, y, z}
float accelerometer[3]; // {x, y, z}
float gyroscope[3]; // {x, y, z}

String data;
String groundData;

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Function Decleration //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void setup();
void loop();
void pressureToAltitudeSetup();
void pressureToAltitude();
void updateData();
void blinkSequence();
void Blink(int led, bool ledVal);
void buzzerSequence();

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// SETUP /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin(serialBAUD); //define baud rate in variable decleration above
  
  pinMode(fixLED,OUTPUT); 
  pinMode(xbeeLED,OUTPUT);
  pinMode(statusLED,OUTPUT);
  pinMode(uniqueLED,OUTPUT);
  pinMode(13,OUTPUT);

  imuSetup();
  updateIMU();

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
  if(magnetBootup) magnetometerBootup();
  if(ppod) ppodSetup();
  if(rfd900) rfd900Setup();
  logData(header);

  buzzerSequence();  
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LOOP ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

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
  float pressurePSF = (pressureOnePSI*144);
  
  float altFt = -1.0;
  //UNCOMMENT WHEN RELIABLE PRESSURE SENSORS ARE ON BOARD
  /*if (pressurePSF > pressureBoundary1)// altitude is less than 36,152 ft ASL
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
  else{altFt = -1.0;}*/
  
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
  delay(150);
  noTone(38);
  delay(20);
  tone(38,2450);
  delay(150);
  noTone(38);
  delay(20);
  tone(38,2450);
  delay(150);
  noTone(38); 
}
