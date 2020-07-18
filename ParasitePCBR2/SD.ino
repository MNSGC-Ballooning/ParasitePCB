#include <SD.h>

#define chipSelect BUILTIN_SDCARD //Should highlight if you have teensy 3.5/3.6/4.0 selected

File datalog;
char filename[] = "SDPPOD00.csv"; //File name template
bool sdActive = false;

void sdSetup(){
  pinMode(chipSelect,OUTPUT);
  if(!SD.begin(chipSelect)){
    Serial.println("Card failed, or not present");
    for(int i=1; i<25; i++){
      digitalWrite(13,HIGH);
      delay(200);
      digitalWrite(13,LOW);
      delay(200);
    }
  }
  else {
    Serial.println("Card initialized.\nCreating File...");
    for (byte i = 0; i < 100; i++) {
      filename[6] = '0' + i/10; 
      filename[7] = '0' + i%10; //filename must be x characters for this to work ("filename[x] = ")
      if (!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        sdActive = true;
        Serial.println("Logging to: " + String(filename));
        break;}
    }
    if (!sdActive) Serial.println("No available file names; clear SD card to enable logging");
  } 
}

void logData(String Data){
  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(Data);
  datalog.close();
  Serial.println(Data);
}
