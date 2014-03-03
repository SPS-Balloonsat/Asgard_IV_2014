#include <I2Cdev.h>
/*
SPS Balloonsat 2014 - ASGARD-IV
MPU-6050 non-integrated motion sensing code
Based off Jeff Rowberg's i2cdevlib library and example code (which is MIT licensed)
NB This code relies upon the MPU6050_6Axis_MotionApps20.h file
having been altered such that line 305 reads 0x02,   0x16,   0x02,   0x00, 0x1E               // D_0_22 inv_set_fifo_rate
(the last one being the altered number)
This drops the FIFO IN/OUT rate to only around 5-6Hz, which allows code doing other stuff to keep up.


CHANGELOG: (also see git)
2014-1-30:  Created file. Added most initial config code. 
2014-2-4: Finished off roughly. Needs some tweaking. Not too bad though; seems to work.
2014-2-28: Added some offsets created by calibration program available on forum. Seems to be roughly accurate. Added averaging mechanism for accelerometer.
2014-2-1: Added gyro averaging. Seems to work. NB TODO: finish accel max/min system.
2014-2-2: Added accel max/min. Should be ready for integration.
2014-2-3: Tested on T-Minus
*/
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
//wire library for i2c + MPU libraries

MPU6050 mpu;//where it all happens!
//Taken from example code
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int fifoCount;
int packetSize;

int xAccelMax, xAccelMin, xAccelAvg, xAccelSampCount;
int yAccelMax, yAccelMin, yAccelAvg, yAccelSampCount;
int zAccelMax, zAccelMin, zAccelAvg,zAccelSampCount;//to be used for the purposes suggested by the names!
int xGyroAvg, yGyroAvg, zGyroAvg, xGyroSampCount,yGyroSampCount,zGyroSampCount;
unsigned long averageTimer = 0;
#define averagePeriod 8500
volatile boolean mpuInterrupt=false;
boolean mpuIntStatus = false;

//using the same variables makes life easier with the actual DMP motion reading section.

void dmpDataReady(){
  mpuInterrupt = true;  //same code as i2cdevlib
}

void setup(){
  Wire.begin();//Join i2C bus and desktop serial bus
  Serial.begin(9600);
  while(!Serial.available())  ;//wait for the user to open the serial monitor (to be removed in the final edition)<<<<<<<<<<<<<<<<<<<=================================================================================
  mpu.initialize();
 
  if(!mpu.testConnection())
    Serial.println("Error - MPU6050 not found.");
  else
    Serial.println("Online.");
 
  int dmpConfigStatus = mpu.dmpInitialize();
//Calibration code offsets; -2523	396	1266	-11	-8	13
    mpu.setXAccelOffset(-2518);
    mpu.setYAccelOffset(395);
    mpu.setZAccelOffset(1267);
    mpu.setXGyroOffset(-11);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(14);
    
    
    if(dmpConfigStatus == 0){
      Serial.println("Configuration success.");
      //Consider using interrupts - can we afford them with everything else going on?
      //On my Arduino Leonardo, interrupt pin 0 is actually one of the i2c pins, so we'll use interrupt pin 2 (pin 0)
      attachInterrupt(2, dmpDataReady, RISING);
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else{
      if(dmpConfigStatus == 1)
        Serial.println("DMP initial memory load failed - couldn't program.");
      else if(dmpConfigStatus == 2)
        Serial.println("DMP config updates failed.");
    }  
        Serial.println("Enabling DMP");//It transpires that this is important. Can't think why.
     mpu.setDMPEnabled(true);
     averageTimer = millis();

}

void loop(){
    //other code here.
    if ((millis()-averageTimer) >= averagePeriod){
      //round up averages
      //store instead of Serial
      xAccelAvg = (int)(xAccelAvg / xAccelSampCount);
      yAccelAvg = (yAccelAvg / yAccelSampCount);
      zAccelAvg = (zAccelAvg / zAccelSampCount);
      xGyroAvg = (xGyroAvg / xGyroSampCount);
      yGyroAvg = yGyroAvg / yGyroSampCount;
      zGyroAvg = zGyroAvg / zGyroSampCount;
      Serial.println(xAccelMax);
      Serial.println(yAccelMin);
      Serial.println(zAccelMax);
      Serial.println("=========================");
     averageTimer = millis();
     xAccelAvg = 0;
     yAccelAvg = 0;
     zAccelAvg = 0;
     xAccelSampCount = 0;
     yAccelSampCount = 0;
     zAccelSampCount = 0;
     xGyroAvg = 0;
     yGyroAvg = 0;
     zGyroAvg = 0;
     xGyroSampCount = 0;
     yGyroSampCount = 0;
     zGyroSampCount = 0;     
     xAccelMax = 0;
     yAccelMax = 0;
     zAccelMax = 0;
     xAccelMin = 0;
     yAccelMin = 0;
     zAccelMin = 0;
     }
    
    
    
    
  //=======================================================================================================================================================
  if(mpuInterrupt == true){
    mpuInterrupt = false;    
                     //next section lifted directly from example code incl. comments.  Some comments are added for clarity.=============================
       // reset interrupt flag and get INT_STATUS byte
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
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
/*
        #ifdef OUTPUT_READABLE_QUATERNION
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
        #endif
*/
  //      #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            ypr[0] *= 180/M_PI;
            ypr[1] *= 180/M_PI;
            ypr[2] *= 180/M_PI;

            xGyroAvg += ypr[0];
            yGyroAvg += ypr[1];
            zGyroAvg += ypr[2];
            xGyroSampCount++;
            yGyroSampCount++;
            zGyroSampCount++;
            
//        #endif

     //   #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  /*          Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);*/
            //averaging mechanism.
            if(xAccelMax == 0 && yAccelMax == 0 && zAccelMax == 0){//if these value havese mu been reset
              xAccelMax = aaReal.x;  //then the
              yAccelMax = aaReal.y;
              zAccelMax = aaReal.z;
              xAccelMin = aaReal.x;
              yAccelMin = aaReal.y;
              zAccelMin = aaReal.z;
            }
            if(xAccelMax < aaReal.x)
              xAccelMax = aaReal.x;
            if(yAccelMax < aaReal.y)
              yAccelMax = aaReal.y;
            if(zAccelMax < aaReal.z)
              zAccelMax = aaReal.z;
            if(xAccelMin > aaReal.x)
              xAccelMin = aaReal.x;
            if(yAccelMin > aaReal.y)
              yAccelMin = aaReal.y;
            if(zAccelMin > aaReal.z)
              zAccelMin = aaReal.z;   
              
            xAccelAvg += aaReal.x;
            yAccelAvg += aaReal.y;
            zAccelAvg += aaReal.z;
            xAccelSampCount++;
            yAccelSampCount++;
            zAccelSampCount++;
            
     /*   #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
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
        #endif
    
*/
        //theoretically that should be all of the DMP reading code required. Now I just need to develop some sort of logging solution - possibly taking a DMP gyro reading every second, or as frequently
        //as feasible, and an accel max reading every second? (on each axis.) That'll produce rather a lot of data after a multi-hour flight - we really must look at storage, and soon.
    }

}}
