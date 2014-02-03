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
//Taken from example code
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//using the same variables makes life easier with the actual DMP motion reading section.

volatile bool dmpDataReady(){
  mpuInterrupt = true;  //same code as i2cdevlib
}

void setup(){
  Wire.begin();//Join i2C bus and desktop serial bus
  Serial.begin(9600);
  delay(1000);//wait for the user to open the serial monitor (to be removed in the final edition)<<<<<<<<<<<<<<<<<<<=================================================================================
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
      //On my Arduino Leonardo, interrupt pin 0 is actually one of the i2c pins, so we'll use interrupt pin 2 (pin 0)
      attachInterrupt(0, dmpDataReady, RISING);
    }
    else{
      if(dmpConfigStatus == 1)
        Serial.println("DMP initial memory load failed - couldn't program.");
      else if(dmpConfigStatus == 2)
        Serial.println("DMP config updates failed.");
    }  
    
    }

void loop(){
  if(mpuInterrupt == true){
    mpuInterrupt = false;    
                                                          //next section lifted directly from example code incl. comments.  Some comments are added for clarity.=============================
       // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();  //I'm guessing we should aim to have this at around thirty-forty to avoid excessive polling freq, which could interfere w/ the gamma ray 
                                     //  pulse counting section, which should also use interrupts.

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));  //"Sound the alarum bell!" If this happens, we probably need to look at reducing dmp frequency, or possibly increasing fifo buffer size.
                                                //                            Otherwise we'll just have to improve the speed of the other code in the main loop.

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);  //read out all of the FIFO data available.
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

  /*      #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif */

     //   #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
   //     #endif

    /*    #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
       #endif */

   //     #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
    //    #endif
        //theoretically that should be all of the DMP reading code required. Now I just need to develop some sort of logging solution - possibly taking a DMP gyro reading every second, or as frequently
        //as feasible, and an accel max reading every second? (on each axis.) That'll produce rather a lot of data after a multi-hour flight - we really must look at storage, and soon.
    }
}

