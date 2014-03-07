#include <Wire.h>

/* IST HYT-271 sketch
SPS Balloonsat 2014
I have based this roughly on the C-language sketch found at ist-ag.com (see readme link)

============================================
Changelog:

14-3-6: Created rough outline. Have not tested, but compiles well to ~3kb.

*/
#define hytAddr 0x28
float temp, hum;


void setup(){
  Wire.begin();//Join i2c bus
  Serial.begin(9600);//set up serial for debugging
  
  
}

void loop(){}

void send_HYT_MR(){//send the instruction to start taking a measurement
//needs a 60ms delay before reading
  Wire.beginTransmission(hytAddr);
  Wire.write(0);//I !!THINK!! this can be basically anything.
  Wire.endTransmission();
}

void read_HYT_MR(byte data[5]){
  Wire.requestFrom(hytAddr,5);//V Important - we don't wait for wire available, 
  //in case this causes a hang.
  while(Wire.available()){
    for(int i = 0;i<5;i++){
      data[i] = Wire.read();//read the expected five bytes into an array.
    }
  }
}

boolean handle_HYT_data(byte data[5]){
  
  if(((data[1] & 0x40)>>6) == 0){//if the data is new (status bits)
    unsigned int temp_raw = (data[3] << 8) | (data[4] >> 2);
    
    unsigned int hum_raw = ((data[1] & 0x3F)<<8) | data[2];
    
    if (temp_raw < 0x3FFF && hum_raw < 0x3FFF){//if values within normal limits
      temp = ((float)(temp_raw)*165.0F / 16383.0F) - 40.0F;
      hum = (float)hum_raw * 100.0F / 16383.0F;
    }
    //Otherwise, something is kaputt.
    
  }
}
