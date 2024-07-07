#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Servo leftwheel, rightwheel;

// PID constants for angle control
float Kp_angle = 0.82;
float Ki_angle = 0.0;
float Kd_angle = 0.61;

// PID constants for speed control
float Kp_speed = 1.0;
float Ki_speed = 0.0;
float Kd_speed = 1.0;

// Variables for angle PID
float angle_error = 0, angle_previous_error = 0, angle_integral = 0;

// Variables for speed PID
float speed_error = 0, speed_previous_error = 0, speed_integral = 0;

float current_speed = 0;
float target_speed = 3.0;  // Desired forward speed

const int SERVO_STOP = 90;
const int SERVO_RANGE = 90;

// Motor pins
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;

const float MAX_LEAN_ANGLE = 3.0; // Maximum lean angle in degrees
const float MAX_SPEED_OUTPUT = 5.0;

// Calibration offsets
float offsetX = 0, offsetY = 0, offsetZ = 0;

void setup() {
  Serial.begin(9600);
  
  if(!accel.begin()) {
    Serial.println("No ADXL345 detected. Check wiring!");
    while(1);
  }
  
  accel.setRange(ADXL345_RANGE_2_G);
  
  // Perform calibration
  calibrateADXL345();
  
  // Set up motor pins
  leftwheel.attach(LEFT_MOTOR_PIN);
  rightwheel.attach(RIGHT_MOTOR_PIN);
}

void loop() {
  sensors_event_t event; 
  accel.getEvent(&event);
  
  // Apply calibration offsets
  float calibratedX = event.acceleration.x - offsetX;
  float calibratedY = event.acceleration.y - offsetY;
  float calibratedZ = event.acceleration.z - offsetZ;
  
  // Calculate tilt angle
  float tilt_angle = atan2(-calibratedY, calibratedZ) * 180 / PI;
  
  // Calculate current speed (simple approximation)
  current_speed += (calibratedX * 0.01);  // Integrate acceleration to get speed
  
  // Speed PID
  speed_error = target_speed - current_speed;
  speed_integral += speed_error;
  speed_integral = constrain(speed_integral, -100, 100);  // Prevent integral windup
  float speed_derivative = speed_error - speed_previous_error;
  float speed_output = Kp_speed * speed_error + Ki_speed * speed_integral + Kd_speed * speed_derivative;
  speed_previous_error = speed_error;
  
  // Calculate target angle based on speed output
  float target_angle = calculateTargetAngle(speed_output);
  
  // Angle PID
  angle_error = tilt_angle - target_angle;
  angle_integral += angle_error;
  angle_integral = constrain(angle_integral, -100, 100);  // Prevent integral windup
  float angle_derivative = angle_error - angle_previous_error;
  float angle_output = Kp_angle * angle_error + Ki_angle * angle_integral + Kd_angle * angle_derivative;
  angle_previous_error = angle_error;
  
  // Combine outputs
  float left_output = angle_output;
  float right_output = angle_output;
  
  // Drive motors
  driveMotors(left_output, right_output);
}

void driveMotors(float left_output, float right_output) {
  int left_servo_value = SERVO_STOP + constrain(left_output, -SERVO_RANGE, SERVO_RANGE);
  int right_servo_value = SERVO_STOP - constrain(right_output, -SERVO_RANGE, SERVO_RANGE);
  
  leftwheel.write(left_servo_value);
  rightwheel.write(right_servo_value);
}

float calculateTargetAngle(float speed_output) {
    // Example of a more complex relationship
    float normalized_output = speed_output / MAX_SPEED_OUTPUT;
    return MAX_LEAN_ANGLE * tanh(normalized_output);
}

void calibrateADXL345() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int numSamples = 100;
  
  Serial.println("Calibrating... Keep the sensor still and level.");
  
  for(int i = 0; i < numSamples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    
    sumX += event.acceleration.x;
    sumY += event.acceleration.y;
    sumZ += event.acceleration.z;
  }
  
  offsetX = sumX / numSamples;
  offsetY = sumY / numSamples;
  offsetZ = sumZ / numSamples - 9.8; // Subtract 1g from Z-axis
  
  Serial.println("Calibration complete!");
  Serial.print("X offset: "); Serial.println(offsetX);
  Serial.print("Y offset: "); Serial.println(offsetY);
  Serial.print("Z offset: "); Serial.println(offsetZ);
}
