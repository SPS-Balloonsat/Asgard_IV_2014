float reading, pressure;
#define pressPin A0


void setup(){
  
  Serial.begin(9600);
  pinMode(A0, INPUT);
  Serial.println("Starting readings.");
}
void loop(){
  reading = analogRead(A0);
  reading *= (float)(5/1024.0);//Vout in volts
  pressure = (float)(((float)(reading / 5) + 0.095) / 0.0009 );
  Serial.println(pressure);
  Serial.println("===================");
  delay(50);
}
