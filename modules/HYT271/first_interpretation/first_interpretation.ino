#include <Wire.h>

/* IST HYT-271 sketch
SPS Balloonsat 2014
I have based this roughly on the C-language sketch found at ist-ag.com (see readme link)

============================================
Changelog:

14-3-6: Created rough outline. Have not tested, but compiles well to ~3kb.
14-3-8: Fixed numerous errors. Now working well, but binary size 8.7kb w/ serial strings. 
*/
#define hytAddr 0x28
float temp, hum;
byte data[4];


void setup(){
  Wire.begin();//Join i2c bus
  Serial.begin(9600);//set up serial for debugging
  send_HYT_MR();//We'll see if we can eliminate this delay in the final copy.  
  
}

void loop(){

  read_HYT();
    send_HYT_MR();
  Serial.println("===================");
  handle_HYT_data();
Serial.println(hum);Serial.println(temp);
delay(500);

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

