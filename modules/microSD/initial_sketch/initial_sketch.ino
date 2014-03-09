#include <Wire.h>

#include <SPI.h>
#include <SD.h>
byte data[4];
#define hytAddr 0x28

/*
SPS Balloonsat 2014 - ASGARD_IV

SD Data-logging code

To use with the Adafruit Micro-SD shield. (NB this includes logic-level translation)

Intention here is to test system by logging some data.
===============================================================
Changelog:
2014-8-3: Created file.

*/

#define selectPin 10
float hum, temp;

File currentLogFile;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  send_HYT_MR();
  Serial.println("Attempting to initialise microSD Card");
  pinMode(SS, OUTPUT);
  
  if(SD.begin(selectPin))
    Serial.println("Card successfully initialised.");
  else
    Serial.println("Initialisation failure.");//there doesn't seem much point in doing anything else in this eventuality!
  currentLogFile = SD.open("datalog.csv", FILE_WRITE);
  currentLogFile.write("Time,Temp,Hum \n");//write the header columns
  currentLogFile.close();  

}
unsigned long lastLogTime = millis();
void loop(){

  if(((millis()/1000)%10) == 0 && (millis()/1000) != lastLogTime){//log every ten seconds
  read_HYT();
  handle_HYT_data();
    //Do Datalogging! Hooray!
     currentLogFile = SD.open("datalog.csv", FILE_WRITE);
     if(!currentLogFile)
       Serial.println("error");
    lastLogTime = (millis()/1000);  
    currentLogFile.print(millis());
    currentLogFile.print(",");
    currentLogFile.print(temp);
    currentLogFile.print(",");
    currentLogFile.println(hum);
    Serial.println(temp);
    Serial.println(hum);
    Serial.println("========");
    currentLogFile.close();  
  send_HYT_MR();
  }
  delay(10);
}


void send_HYT_MR(){//send the instruction to start taking a measurement
//needs a 60ms delay before reading
  Wire.beginTransmission(hytAddr);
  Wire.endTransmission();
}

void read_HYT(){
  
  Wire.requestFrom(hytAddr,4);//V Important - we don't wait for wire available, 
  //in case this causes a hang.
    for(byte i = 0;i<4;i++){
      data[i] = Wire.read();//read the expected five bytes into an array.
    }
    
    Wire.endTransmission();
}

void handle_HYT_data(){
  int rawH = ((data[0] << 8) & 0x3FFF) | data[1]; 

  
  int rawT = data[2] << 6;
  rawT = rawT | (data[3] >> 2);
  if(rawT < 0x3FFF && rawH < 0x3FFF){
    temp = ((float)(rawT) * 165.0 / 16384.0) - 40.0;
    hum = (float)rawH * 100.0 / 16384.0;
    
  }
  else{
    Serial.println("Data out of range.");
  }
}

