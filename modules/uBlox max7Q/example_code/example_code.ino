/*
SPS Balloonsat 2014.
GPS interface code w/ tinyGPS. Adapted from example code from HAB Supplies - see below.
Changelog:
2014-3-16

  GPS Level Convertor Board Test Script
  03/05/2012 2E0UPU
  For use with the HAB Supplies Level Convertor Board
  http://ava.upuaut.net/store
 
  Initialise the GPS Module in Flight Mode and then echo's out the NMEA Data to the Software Serial1.
 
  This example code is in the public domain.
  Additional Code by J Coxon (http://ukhas.org.uk/guides:falcom_fsa03)
 */
#include <TinyGPS++.h>
 
TinyGPSPlus gps;

 
 
byte gps_set_sucess = 0 ;
 
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("GPS Level Convertor Board Test Script");
  Serial.println("03/06/2012 2E0UPU");
  Serial.println("Initialising....");
 while(!Serial.available());
 // THIS COMMAND SETS MODE AND CONFIRMS IT 
  Serial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {  //Portable mode
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0F, 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84                         
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
 
}
 
void loop()
{
  delay(200);
  while(Serial1.available()) {
    char inByte = Serial1.read();
    gps.encode(inByte);
  }
  
  Serial.print("Time:"); 
  Serial.println(gps.time.value());    
  if(true){
  Serial.print("Lat/long:"); 
  Serial.print(gps.location.lat());
  Serial.print(", ");
  Serial.println(gps.location.lng());
  Serial.print("Horizontal accuracy:");
  Serial.println(gps.hdop.value() / 100);
  }
  else Serial.println("Data not valid.");
}
  
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
