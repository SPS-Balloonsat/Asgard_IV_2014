/* 
 * TimeSerial.pde
 * example code illustrating Time library set through serial port messages.
 *
 * Messages consist of the letter T followed by ten digit time (as seconds since Jan 1 1970)
 * you can send the text on the next line using Serial Monitor to set the clock to noon Jan 1 2013
 T1357041600  
 *
 * A Processing example sketch to automatically send the messages is inclided in the download
 * On Linux, you can use "date +T%s > /dev/ttyACM0" (UTC time zone)
 */ 
 
#include <Time.h>  
#include <SPI.h>
#include <SD.h>


#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define selectPin 10
float humidity, temperature;

File currentLogFile;

void setup(){
  Serial.begin(9600);

  Serial.println("Attempting to initialise microSD Card");
  pinMode(SS, OUTPUT);
  
  if(SD.begin(selectPin))
    Serial.println("Card successfully initialised.");
  else
    Serial.println("Initialisation failure.");//there doesn't seem much point in doing anything else in this eventuality!
    
  currentLogFile = SD.open("data_hum"+"123"+".csv", FILE_WRITE);
  currentLogFile.write("Time,Temp,Hum \n");//write the header columns
  
  setSyncProvider( requestSync);  //set function to call when sync required
  Serial.println("Waiting for sync message");while(!Serial.available);
  while(Serial.available())
    processSyncMessage();
    
}
byte minute_logged;
void loop(){    
  if(second() == 30 && minute_logged != minute()){
    minute_logged = minute(); 
    String outputString = now() + ",";

    outputString += temperature;
    outputString += ",";
    outputString += humidity;
    outputString += "\n";
    currentLogFile.write(outputString);
    
  }
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

