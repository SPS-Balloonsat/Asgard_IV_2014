/*
SPS Balloonsat 2014
 Initial integration. 
 Codename: "Arctic Char"
 Compiled size ~42kb.
 
 Changelog:
 2014-3-14: Created file. Added MPU library
 2014-3-15: Added HYT-271 functionality.
 2014-3-18: Added GPS. Removed potential for infinite loop in GPS setup if connection faulty. Began SD integration.
 2014-3-19: SD card integration.
 2014-3-25: Tweaks prior to launch.
 2014-3-26: Added pressure sensor code (last minute). For use w/ potential divider from MPL4115, using 5k/10k resistors (top/bottom). 
 Changed GPS config code to flight dynamic mode (>4G).
 
 */
//I2C library
#include <Wire.h>

//General definitions =========================================================================
#define averagePeriod 2000

//SD libraries =========================================================================
#include <SPI.h>
#include <SD.h>

//GPS library =========================================================================
#include <TinyGPS++.h>

//GPS definition =========================================================================
TinyGPSPlus gps;

//MPU libraries =========================================================================
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

//SD definitions =========================================================================
File currentLogFile;
#define selectPin 10

//Pressure definition
#define pressurePin A6 //pin 20
float pressure, pressure_reading;

//Gamma defs
#define gammaPin 22
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

#define mpuInterruptNumber 5
volatile boolean mpuInterrupt=false;
boolean mpuIntStatus = false;

//MPU ISR
void dmpDataReady(){
  mpuInterrupt = true;  //same code as i2cdevlib

}
//AverageTimer declaration ============================================================
unsigned long averageTimer = 0;

// ===============================================
void setup(){
  Wire.begin();//Join i2C bus and desktop serial bus
  Serial.begin(38400);
  pinMode(pressurePin, INPUT);

  Serial.println("Starting readings.");

  //SD card init code ============================================================
  pinMode(SS, OUTPUT);
  delay(500);//Wait for power supply to stabilise
  if(SD.begin(selectPin))
    Serial.println("Card successfully initialised.");
  else
    Serial.println("Initialisation failure.");//there doesn't seem much point in doing anything else in this eventuality!   
    
 while(!currentLogFile){
  currentLogFile = SD.open("a.csv", FILE_WRITE);//create log file (or reopen)
  if(!currentLogFile)Serial.println("Error.");
 }
  currentLogFile.write("gps_lat,gps_long,gps_date,gps_time,gps_SC,gps_speed,gps_course,gps_alt,temp,hum");
  //gps_lat, gps_long, gps_date, gps_time, gps_status, accMaX, accMaY, accMaZ, accMiX, accMiY, accMiz, accAvX, accAvY, accAvZ, gyrAvX, gyrAvY, gyrAvZ, press, temp, hum, gammaCPS
  currentLogFile.write(",accMaX,accMaY,accMaZ,accMiX,accMiY,accMiz,accAvX,");
  currentLogFile.write("accAvY,accAvZ,gyrAvX,gyrAvY,gyrAvZ,gammaCPS,sampTime,pressure \n");
  currentLogFile.close();//Write headers and save.
  Serial.println("SD datalogging headers written.");

  //GPS Init code ==================================================================================
  Serial1.begin(9600);
  // THIS COMMAND SETS MODE AND CONFIRMS IT 
  Serial.println("Setting GPS nav mode: ");
  uint8_t setNav[] = {  //Flight mode
    0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x08,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4F,0x1F,0xB5,0x62,0x06,0x24,0x00,0x00,0x2A,0x84};
  byte loop_counter = 0;
  while(!gps_set_sucess && loop_counter < 15)
  {
    loop_counter++;
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
    
  }
  gps_set_sucess=0;
  Serial.println("GPS done.");
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
//=========================================================================================
//sampling code
  if ((millis()-averageTimer) >= averagePeriod){//averaging code
  unsigned long timeDifference = millis() - averageTimer;
  Serial.println("Logging.");
    //GPS - get date, time, long, lat, course, speed ==================================================
    //gps_lat,gps_long,gps_date,gps_time,gps_SC,gps_speed,gps_course,gps_alt 
      currentLogFile = SD.open("a.csv", FILE_WRITE);
      if(!currentLogFile)Serial.println("Error.");
      currentLogFile.write(" ");//initiate write. Don't know why
      currentLogFile.print(gps.location.lat()*1000000);currentLogFile.print(",");//millionths
      currentLogFile.print(gps.location.lng()*1000000);currentLogFile.print(",");//millionths
      currentLogFile.print(gps.date.value());currentLogFile.print(",");//date, time DDMMYY SSMMHH unsigned ints (int 32)
      currentLogFile.print(gps.time.value());currentLogFile.print(",");
      currentLogFile.print(gps.satellites.value());currentLogFile.print(",");//no of satellites - i32
      currentLogFile.print(gps.speed.knots());currentLogFile.print(",");//speed in kts - double (=float)
      currentLogFile.print(gps.course.value());currentLogFile.print(",");//course in 1/100ths degree - i32
      currentLogFile.print(gps.altitude.feet());currentLogFile.print(",");//alt in feet (double)   
      currentLogFile.flush();//save this data
    
    //humidity detection code ==============================================
    read_HYT();
    handle_HYT_data();
    send_HYT_MR();
    //print hum and temp
    currentLogFile.print(temp);currentLogFile.print(",");//temp,hum
    currentLogFile.print(hum);currentLogFile.print(",");
    currentLogFile.flush();
    //===============================================================================
    //MPU averaging code.
    //round up averages
//  currentLogFile.write(",accMaX,accMaY,accMaZ,accMiX,accMiY,accMiz,accAvX,");  <<< headers for reference
//  currentLogFile.write("accAvY,accAvZ,gyrAvX,gyrAvY,gyrAvZ,gammaCPS \n");

    currentLogFile.print(xAccelMax);currentLogFile.print(",");
    currentLogFile.print(yAccelMax);currentLogFile.print(",");
    currentLogFile.print(zAccelMax);currentLogFile.print(",");Serial.println(zAccelMax);
    currentLogFile.print(xAccelMin);currentLogFile.print(",");
    currentLogFile.print(yAccelMin);currentLogFile.print(",");
    currentLogFile.print(zAccelMin);currentLogFile.print(",");
    currentLogFile.flush();
    currentLogFile.print((xAccelAvg / xAccelSampCount));currentLogFile.print(",");
    currentLogFile.print((yAccelAvg / yAccelSampCount));currentLogFile.print(",");
    currentLogFile.print((zAccelAvg / zAccelSampCount));currentLogFile.print(",");
    currentLogFile.print((xGyroAvg / xGyroSampCount));currentLogFile.print(",");
    currentLogFile.print((yGyroAvg / yGyroSampCount));currentLogFile.print(",");
    currentLogFile.print((zGyroAvg / zGyroSampCount));currentLogFile.print(",");
    
    //Gamma photon count
    currentLogFile.print(gammaCount);currentLogFile.print(",");
    currentLogFile.print(timeDifference);currentLogFile.print(",");
    
    //pressure ==============================================================
    pressure_reading = analogRead(pressurePin);
    pressure_reading *= (float)(5/1024.0);//Vout in volts
    pressure = (float)(((float)(pressure_reading / 5) + 0.095)/ 0.0009);
    currentLogFile.println(pressure);   
    
    currentLogFile.close();//save and go!

    xAccelAvg = 0;//Zero values
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
    averageTimer = millis();//reset timer
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
