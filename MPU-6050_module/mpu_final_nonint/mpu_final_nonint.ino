/*
SPS Balloonsat 2014 - ASGARD-IV
MPU-6050 non-integrated motion sensing code
Based off Jeff Rowberg's i2cdevlib library and example code (which is MIT licensed)

CHANGELOG: (also see git)
2014-1-30:  Created file. Added most initial config code. 

*/
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050_9Axis_MotionApps41.h>
#include <MPU6050.h>
#include <helper_3dmath.h>
//wire library for i2c + MPU libraries

MPU6050 mpu;//where it all happens!

void setup(){
  Wire.begin();//Join i2C bus and desktop serial bus
  Serial.begin(9600);
  mpu.initialize();
  if(!mpu.testConnection())
    Serial.println("Error - MPU6050 not found.");
  else
    Serial.println("Online.");
 
  int dmpConfigStatus = mpu.dmpInitilize();
  //now to set gyro offsets - currently unknown. Code copied/pasted from example for now
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); //Don't know what these'll be for us.
    
    if(dmpConfigStatus == 0){
      Serial.println("Configuration success.");
      //Consider using interrupts - can we afford them with everything else going on?
    }
    else{
      if(dmpConfigStatus == 1)
        Serial.println("DMP initial memory load failed - couldn't program.");
      else if(dmpConfigStatus == 2)
        Serial.println("DMP config updates failed.");
    }  
    
    }

void loop(){}

