/*
SPS Balloonsat 2014
Gamma Ray Photon pulse counter
Changelog:
2014-3-13: Created initial file, added rough code. As yet untested.
*/
#define gammaIntPin 22

volatile unsigned long photonHits;
unsigned long sampleTime;

void setup(){
  Serial.begin(9600);
 // while(!Serial.available());
  pinMode(gammaIntPin, INPUT_PULLUP);
  attachInterrupt(gammaIntPin, incGamma, FALLING);
  sampleTime = millis();//sample timer
}

void loop(){
  if(((millis() - sampleTime)) > 1000){
    Serial.println(photonHits);
    Serial.println("==================");
    photonHits = 0;
    sampleTime = millis();
  }  
}

void incGamma(){
  photonHits++;//ISR routine to be kept as short as possible
}
