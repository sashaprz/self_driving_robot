#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// class name (from lib); instance name; constructor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); 

//offset variables
float offsetX; 
float offsetY; 
float offsetZ; 

float calibratedX, calibratedY, calibratedZ; 

Servo rightwheel; 
Servo leftwheel; 

const int leftpin = 9; 
const int rightpin = 10; 

//PID constance
float Kp = 0.3; 
float Ki = 0.0; 
float Kd = 0.25; 

float targetAngle = 0.0; 
float error = 0, prevError = 0, integral = 0, derivative = 0;

void setup() {
  Serial.begin(9600); 

  if (!accel.begin()) {
    Serial.println("No ADXL345 detected, check your wiring"); 
    while(1);
  }

  accel.setRange(ADXL345_RANGE_2_G); 

  calibrate(); 
  
  Serial.print("X offset: "); Serial.println(offsetX); 
  Serial.print("Y offset: "); Serial.println(offsetY); 
  Serial.print("Z offset: "); Serial.println(offsetZ); 

  leftwheel.attach(leftpin); 
  rightwheel.attach(rightpin); 

}

void loop() {
  sensors_event_t event; 
  accel.getEvent(&event);

  calibratedX = event.acceleration.x - offsetX; 
  calibratedY = event.acceleration.y - offsetY; 
  calibratedZ = event.acceleration.z - offsetZ; 

  const float GRAVITY = 9.81; 
  float normalizedZ = calibratedZ;
  float tiltAngle = atan2(-calibratedY, calibratedZ) * 180 / PI;
 
  float scaledTiltAngle = tiltAngle;

  float integralLimit = 20; // Adjust as needed
  Ki = constrain(Ki, -integralLimit, integralLimit);
  
  // PID control
  error = scaledTiltAngle - targetAngle;
  integral += error;
  derivative = error - prevError;

  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Constrain output to motor range
  output = constrain(output, -255, 255);

  Serial.print("Output: "); Serial.println(output);

  drivemotors(output); 

  error = prevError; 
}

void calibrate() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int numSamples = 100;
  
  Serial.println("Calibrating... Keep the sensor still and level.");
  
  for(int i = 0; i < numSamples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    
    sumX += event.acceleration.x;
    sumY += event.acceleration.y;
    sumZ += event.acceleration.z;
    
    delay(2);
  }
  
  offsetX = sumX / numSamples;
  offsetY = sumY / numSamples;
  offsetZ = sumZ / numSamples - 9.8; // Subtract 1g from Z-axis
}

void drivemotors(float output) {
  int servoValue; 

  servoValue = constrain(output, -90, 90); 

  leftwheel.write(90 + servoValue);
  rightwheel.write(90 - servoValue); 

}
