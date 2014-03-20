/*
SPS Balloonsat 2014
 Initial integration. Build no.: 
 Codename: "Arctic Char"
 
 Changelog:
 2014-3-14: Created file. Added MPU library
 2014-3-15: Added HYT-271 functionality.
 2014-3-18: Added GPS. Removed potential for infinite loop in GPS setup if connection faulty. Began SD integration.
 2014-3-19: SD card integration.
 */
//I2C library
#include <Wire.h>

//General definitions =========================================================================
#define averagePeriod 500

//SD libraries =========================================================================
#include <SPI.h>
#include <SD.h>

//GPS library =========================================================================
#include <TinyGPS++.h>

//GPS definition =========================================================================
TinyGPSPlus gps;
double latitude, longitude, gps_alt, gps_speed; //lat, long, speed, altitude, speed
unsigned int gps_sat_count, gps_date, gps_time, gps_course ;//number of satellites, date, time, course in 1/100ths of a deg
//...alt in cm (saves conversion!)
byte gps_data_retrieved = 0; //gps data state indicator - 0 => success, 1 => only date/time, 2=> total disaster.


//MPU libraries =========================================================================
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

//SD definitions =========================================================================
File currentLogFile;
#define selectPin 10

//Gamma defs
#define gammaPin 3
volatile unsigned int gammaCount;

//GPS definitions =======================================================================
byte gps_set_sucess = 0;

//HYT-271 definitions =========================================================================
#define hytAddr 0x28
float temp, hum;
byte hData[4];

//MPU definitions =======================================================
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
byte dmpConfigStatus;
unsigned long averageTimer = 0;

#define mpuInterruptNumber 0
volatile boolean mpuInterrupt=false;
boolean mpuIntStatus = false;

//MPU ISR
void dmpDataReady(){
  mpuInterrupt = true;  //same code as i2cdevlib

}
// ===============================================
void setup(){
  Wire.begin();//Join i2C bus and desktop serial bus
  Serial.begin(38400);
 
  //SD card init code ============================================================
  pinMode(SS, OUTPUT);
  if(SD.begin(selectPin))
    Serial.println("Card successfully initialised.");
  else
    Serial.println("Initialisation failure.");//there doesn't seem much point in doing anything else in this eventuality!   
 
  currentLogFile = SD.open("datalogging.csv", FILE_WRITE);//create log file (or reopen)
  currentLogFile.write("gps_lat, gps_long, gps_date, gps_time, gps_status");
  //gps_lat, gps_long, gps_date, gps_time, gps_status, accMaX, accMaY, accMaZ, accMiX, accMiY, accMiz, accAvX, accAvY, accAvZ, gyrAvX, gyrAvY, gyrAvZ, press, temp, hum, gammaCPS
  currentLogFile.write(", accMaX, accMaY, accMaZ, accMiX, accMiY, accMiz, accAvX,");
  currentLogFile.write(" accAvY, accAvZ, gyrAvX, gyrAvY, gyrAvZ,");
  currentLogFile.write(" press, temp, hum, gammaCPS \n");
  currentLogFile.close();//Write headers and save.
  Serial.println(F"SD datalogging headers written.");

  //GPS Init code ==================================================================================
  Serial1.begin(9600);
  // THIS COMMAND SETS MODE AND CONFIRMS IT 
  Serial.println("Setting GPS nav mode: ");
  uint8_t setNav[] = {  //Portable mode - see separate text files
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0F, 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84                         
  };
  byte loop_counter = 0;
  while(!gps_set_sucess && loop_counter < 15)
  {
    loop_counter++;
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
  //HYT271 init code ========================================================================================
  send_HYT_MR();
  //MPU6050 init code ========================================================================================
  mpu.initialize();

  if(!mpu.testConnection())
    Serial.println("Error - MPU6050 not found.");
  else
    Serial.println("Online.");

  dmpConfigStatus = mpu.dmpInitialize();
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
    pinMode(mpuInterruptNumber, INPUT);
    attachInterrupt(mpuInterruptNumber, dmpDataReady, RISING);
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
  //gps code - deliciously simple!
  while(Serial1.available())
    gps.encode(Serial1.read());



  if ((millis()-averageTimer) >= averagePeriod){//averaging code
    //GPS - get date, time, long, lat, course, speed
    if(gps.time.isValid() && gps.location.isValid()){//if gps data valid
      latitude = gps.location.lat();//lat, long - doubles
      longitude = gps.location.lng();
      gps_date = gps.date.value();//date, time DDMMYY SSMMHH unsigned ints (int 32)
      gps_time = gps.time.value();
      gps_speed = gps.speed.knots();//speed in kts - double (=float)
      gps_course = gps.course.value();//course in 1/100ths degree - i32
      gps_alt = gps.altitude.feet();//alt in feet (double)   
      gps_sat_count = gps.satellites.value();//no of satellites - i32
      gps_data_retrieved = 0;
    }
    else{
      if(gps.time.isValid()){//if only time reading valid
        gps_date = gps.date.value();
        gps_time = gps.time.value();
        gps_data_retrieved = 1;
      }
      else{//if no data recieved, say so.
        gps_data_retrieved = 2;
      }
    }
    //Gamma photon averaging
    unsigned int gammaAvg = (unsigned int)(gammaCount / (millis()-averageTimer));

    //humidity detection code
    read_HYT();
    handle_HYT_data();
    send_HYT_MR();
    //===============================================================================
    //MPU averaging code.
    //round up averages

    xAccelAvg = (int)(xAccelAvg / xAccelSampCount);
    yAccelAvg = (yAccelAvg / yAccelSampCount);
    zAccelAvg = (zAccelAvg / zAccelSampCount);
    xGyroAvg = (xGyroAvg / xGyroSampCount);
    yGyroAvg = yGyroAvg / yGyroSampCount;
    zGyroAvg = zGyroAvg / zGyroSampCount;
    //Now to do datalogging ==============================================================================
//gps_lat, gps_long, gps_date, gps_time, gps_status, accMaX, accMaY, accMaZ, accMiX, accMiY, accMiz, accAvX, accAvY, accAvZ, gyrAvX, gyrAvY, gyrAvZ, press, temp, hum, gammaCPS
   
    currentLogFile = SD.open("datalogging.csv", FILE_WRITE);
    if(gps_data_retrieved == 0){
    currentLogFile.print(latitude);currentLogFile.print(",");
    currentLogFile.print(longitude);currentLogFile.print(",");}
    else
      currentLogFile.write("0,0,");
    if(gps_data_recieved == 0 || gps_data_received == 1){
    currentLogFile.print(gps_date);currentLogFile.print(",");
    currentLogFile.print(gps_time);currentLogFile.print(",");}
    else{
      currentLogFile.write("0,");
      currentLogFile.print(millis());currentLogFile.print(",");
    }
    currentLogFile.print(gps_data_received);currentLogFile.print(",");
    currentLogFile.print(xAccelAvg);currentLogFile.print(",");
    currentLogFile.print(yAccelAvg);currentLogFile.print(",");
    currentLogFile
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
  //MPU6050 code ======================================================================================================
  if(mpuInterrupt == true){//MPU reading code.
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
    } 
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

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
      //theoretically that should be all of the DMP reading code required. Now I just need to develop some sort of logging solution - possibly taking a DMP gyro reading every second, or as frequently
      //as feasible, and an accel max reading every second? (on each axis.) That'll produce rather a lot of data after a multi-hour flight - we really must look at storage, and soon.
    }
  }  
}

//HYT humidity functions ===============================================================================================
void send_HYT_MR(){//send the instruction to start taking a measurement
  //needs a 60ms delay before reading
  Wire.beginTransmission(hytAddr);
  Wire.endTransmission();
}

void read_HYT(){

  Wire.requestFrom(hytAddr,4);//V Important - we don't wait for wire available, 
  //in case this causes a hang.
  for(byte i = 0;i<4;i++){
    hData[i] = Wire.read();//read the expected five bytes into an array.
  }

  Wire.endTransmission();
}

void handle_HYT_data(){
  int rawH = ((hData[0] << 8) & 0x3FFF) | hData[1]; 


  int rawT = hData[2] << 6;
  rawT = rawT | (hData[3] >> 2);
  if(rawT < 0x3FFF && rawH < 0x3FFF){
    temp = ((float)(rawT) * 165.0 / 16384.0) - 40.0;
    hum = (float)rawH * 100.0 / 16384.0;

  }
  else{
    Serial.println("Data out of range.");
  }
}

//GPS functions - provided w/ example code ============================================================

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial1.write(MSG[i]);
  }
  Serial1.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}

//Gamma ray ISR ===========================================
void incGammaCount(){
  gammaCount++;
}
