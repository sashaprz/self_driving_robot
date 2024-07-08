#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Servo leftwheel, rightwheel;

// PID constants for angle control
float Kp_angle = 0.36;
float Ki_angle = 0.0;
float Kd_angle = 0.30;

// PID constants for speed control
float Kp_speed = 0.0;
float Ki_speed = 0.0;
float Kd_speed = 0.0;

// Variables for angle PID
float angle_error = 0, angle_previous_error = 0, angle_integral = 0;

// Variables for speed PID
float speed_error = 0, speed_previous_error = 0, speed_integral = 0;

float previous_tilt_angle = 0; 
const unsigned long DELTA_TIME = 1; // Time between measurements in milliseconds
unsigned long last_time = 0;

float current_speed = 0;
float target_speed = 4.0;  // Desired forward speed

const int SERVO_STOP = 90;
const int SERVO_RANGE = 90;

// Motor pins
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;

const float MAX_LEAN_ANGLE = 40.0; // Maximum lean angle in degrees
const float MAX_SPEED_OUTPUT = 5.0;

// Calibration offsets
float offsetX = 0, offsetY = 0, offsetZ = 0;

// Balance control parameters
const float TILT_THRESHOLD = 20.0; // Degrees
const float ANGULAR_VELOCITY_THRESHOLD = 50.0; // Degrees per second
const float BASE_MAX_SPEED = 5.0; // Maximum allowed speed

class BalanceController {
public:
    float getMaxAllowableSpeed(float currentTilt, float currentAngularVelocity) {
        float maxSpeed = BASE_MAX_SPEED;
        if (abs(currentTilt) > TILT_THRESHOLD || abs(currentAngularVelocity) > ANGULAR_VELOCITY_THRESHOLD) {
            maxSpeed = BASE_MAX_SPEED * (1 - abs(currentTilt) / MAX_LEAN_ANGLE);
        }
        return max(0.0f, min(maxSpeed, BASE_MAX_SPEED));
    }
};

BalanceController balanceController;

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

  unsigned long current_time = millis();
  if (current_time - last_time >= DELTA_TIME) {
    // Calculate tilt angle
    float tilt_angle = atan2(-calibratedY, calibratedZ) * 180 / PI;
    
    // Calculate angular velocity
    float angular_velocity = (tilt_angle - previous_tilt_angle) / (DELTA_TIME / 1000.0);
    
    // Update previous values
    previous_tilt_angle = tilt_angle;
    last_time = current_time;

    // Get max allowable speed from balance controller
    float maxAllowableSpeed = balanceController.getMaxAllowableSpeed(tilt_angle, angular_velocity);
    
    // Calculate current speed (simple approximation)
    current_speed += (calibratedX * 0.1);  // Integrate acceleration to get speed
    current_speed = constrain(current_speed, -maxAllowableSpeed, maxAllowableSpeed);
    
    // Speed PID
    speed_error = min(target_speed, maxAllowableSpeed) - current_speed;
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
    
    Serial.print("Angle Output: "); Serial.print(angle_output); 
    Serial.print(" Speed Output: "); Serial.print(speed_output);
    Serial.print(" Max Allowable Speed: "); Serial.println(maxAllowableSpeed);

    drivemotors(angle_output); 
  }
} 

void drivemotors(float output) {
  int servoValue; 

  servoValue = constrain(output, -90, 90); 

  leftwheel.write(90 + servoValue);
  rightwheel.write(90 - servoValue); 
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
