#include <NewPing.h>
//this is basically to just see if the sensor is working. im going to integrate it into the actual driving stuff in a bit. 

#define TRIGGER_PIN 3
#define ECHO_PIN 11
#define MAX_DISTANCE 400

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50); 

  unsigned int distance = sonar.ping_cm(); 

  Serial.print("Distance: "); 
  Serial.print(distance); 
  Serial.print(" cm\n");

}
