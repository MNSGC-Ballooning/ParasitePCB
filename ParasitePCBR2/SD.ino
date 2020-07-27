#include <SD.h>

#define chipSelect BUILTIN_SDCARD //Should highlight if you have teensy 3.5/3.6/4.0 selected

File datalog;
File datalogIMU;
char filename[] = "SDCARD00.csv";

bool sdActive = false;

void sdSetup(){
  pinMode(chipSelect,OUTPUT);
  if(!SD.begin(chipSelect)){
    Serial.println("Card failed, or not present");
    updateOled("Turn off\nand Insert SD card");
    for(int i=1; i<20; i++){
      digitalWrite(13,HIGH);
      digitalWrite(sdLED,LOW);
      delay(100);
      digitalWrite(13,LOW);
      digitalWrite(sdLED,HIGH);
      delay(100);
    }
  }
  else {
    Serial.println("Card initialized.\nCreating File...");
    for (byte i = 0; i < 100; i++) {
      filename[6] = '0' + i/10; 
      filename[7] = '0' + i%10;
      if (!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        sdActive = true;
        Serial.println("Logging to: " + String(filename));
        updateOled("Logging:\n\n" + String(filename));
        delay(1000);
        break;}
    }
    if (!sdActive) {
      Serial.println("No available file names; clear SD card to enable logging");
      updateOled("Clear SD!");
      delay(5000);
    }
  } 
}

void logData(String Data){
  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(Data);
  datalog.close();
  Serial.println(Data);
}
