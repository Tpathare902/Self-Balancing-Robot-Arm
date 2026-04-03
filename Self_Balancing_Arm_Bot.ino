 #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// ================================================================
// 1. PIN & OBJECT DEFINITIONS
// ================================================================
// CHANGED: Moved from 0,1 to 2,3 to avoid hardware serial conflicts
SoftwareSerial BT(2, 3);
Servo armServo;          
Servo gripServo;         
Adafruit_MPU6050 mpu;

// Motor Pins (L298N)
int ENA = 6; int IN1 = A2; int IN2 = A3;
int ENB = 5; int IN3 = 9;  int IN4 = 4;

// ================================================================
// 2. SERVO LOGIC 
// ================================================================
int armCurrent = 90;   
int armTarget = 90;    
int gripCurrent = 90;  
int gripTarget = 90;
unsigned long lastServoMove = 0;
int servoDelay = 25;   // Increased to 25ms for smoother/slower arm movement

// ================================================================
// 3. BALANCING & PID SETTINGS (Optimized)
// ================================================================
int Motor_Polarity = -1; 
float Neutral_Angle = -2.0; 
float Kp = 14.5;        // Slightly increased for better response
float Ki = 180.0;        // Integral gain for steady-state error correction
float Kd = 0.4;         // Increased to dampen arm-movement shocks
float Integral_Offset = 0; 
int Min_Speed = 35;     // Higher minimum speed to handle arm weight

// ================================================================
// 4. KALMAN FILTER & SENSOR VARIABLES
// ================================================================
struct KalmanFilter {
  float Q_angle = 0.001; float Q_bias = 0.003; float R_measure = 0.03;
  float angle = 0; float bias = 0; float P[2][2] = {{0,0},{0,0}};
};
KalmanFilter kalmanY;

unsigned long loopTimer = 0;
unsigned long lastCommandTime = 0;
float gyroY_offset = 0.0;
float Pitch = 0.0;
float Drive_Offset = 0.0; 
int Turn_Offset = 0;
float Error = 0.0, PrevError = 0.0, Integral = 0.0;

// Function Level Comments:
//     Function Name: kalmanUpdate
//     Input: KalmanFilter *k (pointer to KalmanFilter struct), float newAngle (new angle measurement), float newRate (new rate measurement), float dt (time delta)
//     Output: float (filtered angle)
//     Logic: Updates the Kalman filter with new measurements and returns the filtered angle.
//     Example Call: float filtered = kalmanUpdate(&kalmanY, accAngle, gyroRate, dt);
// ============ KALMAN UPDATE FUNCTION ============
float kalmanUpdate(KalmanFilter *k, float newAngle, float newRate, float dt) {
  float rate = newRate - k->bias;
  k->angle += dt * rate;
  k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
  k->P[0][1] -= dt * k->P[1][1];
  k->P[1][0] -= dt * k->P[1][1];
  k->P[1][1] += k->Q_bias * dt;
  float S = k->P[0][0] + k->R_measure;
  float K[2] = {k->P[0][0] / S, k->P[1][0] / S};
  k->angle += K[0] * (newAngle - k->angle);
  k->bias  += K[1] * (newAngle - k->angle);
  float P00_temp = k->P[0][0]; float P01_temp = k->P[0][1];
  k->P[0][0] -= K[0] * P00_temp; k->P[0][1] -= K[0] * P01_temp;
  k->P[1][0] -= K[1] * P00_temp; k->P[1][1] -= K[1] * P01_temp;
  return k->angle;
}

// Function Level Comments:
//     Function Name: driveMotors
//     Input: int speed (motor speed), int turn (turn offset)
//     Output: void
//     Logic: Controls the motors based on speed and turn values.
//     Example Call: driveMotors(motorPWM, Turn_Offset);
// ============ MOTOR CONTROL FUNCTIONS ============
void driveMotors(int speed, int turn) {
  int leftSpeed  = (speed + turn) * Motor_Polarity;
  int rightSpeed = (speed - turn) * Motor_Polarity;
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  if (leftSpeed > 0) { analogWrite(ENA, leftSpeed); digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { analogWrite(ENA, -leftSpeed); digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  if (rightSpeed > 0) { analogWrite(ENB, rightSpeed); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  }
}
// Function Level Comments:
//     Function Name: stopMotors
//     Input: none
//     Output: void
//     Logic: Stops all motors.
//     Example Call: stopMotors();
void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// Function Level Comments:
//     Function Name: setup
//     Input: none
//     Output: void
//     Logic: Initializes pins, sensors, and calibrates gyro.
//     Example Call: setup(); // called automatically
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

armServo.attach(11);
gripServo.attach(10);
armServo.write(armCurrent);
gripServo.write(gripCurrent);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  if (!mpu.begin()) { while (1); }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); // Reduced bandwidth to filter out motor noise

  float gyroSum = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    gyroSum += g.gyro.y; delay(2);
  }
  gyroY_offset = gyroSum / 500.0;
  
  sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
  kalmanY.angle = atan(-a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI;
  loopTimer = micros();
}

// Function Level Comments:
//     Function Name: loop
//     Input: none
//     Output: void
//     Logic: Main loop handling Bluetooth commands, servo movement, and balancing.
//     Example Call: loop(); // called automatically
void loop() {
  // --- BLUETOOTH CONTROL ---
  if (BT.available()) {
    char cmd = BT.read();
    lastCommandTime = millis();
    if      (cmd == 'F') { Drive_Offset = -1.5; }
    else if (cmd == 'B') { Drive_Offset = 1.5;  }
    else if (cmd == 'L') { Turn_Offset  = -45;  }
    else if (cmd == 'R') { Turn_Offset  = 45;   }
    else if (cmd == 'S') { Drive_Offset = 0; Turn_Offset = 0; }
    else if (cmd == 'U') { armTarget = 100; } 
    else if (cmd == 'D') { armTarget = 53;  } 
    else if (cmd == 'G') { gripTarget = 90; } 
    else if (cmd == 'O') { gripTarget = 35; } 
  } 
  else if (millis() - lastCommandTime > 100) {
    Drive_Offset = 0; Turn_Offset = 0;
  }

  // --- SMOOTH SERVO MOVEMENT ---
  if (millis() - lastServoMove > servoDelay) {
    if (armCurrent < armTarget) armCurrent++;
    else if (armCurrent > armTarget) armCurrent--;
    if (gripCurrent < gripTarget) gripCurrent++;
    else if (gripCurrent > gripTarget) gripCurrent--;
    armServo.write(armCurrent);
    gripServo.write(gripCurrent);
    lastServoMove = millis();
  }

  // --- BALANCING CALCULATION (250Hz) ---
  unsigned long currentMicros = micros();
  if (currentMicros - loopTimer >= 4000) {
    float dt = (currentMicros - loopTimer) / 1000000.0;
    loopTimer = currentMicros;

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float accAngle = atan(-a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI;
    float gyroRate = (g.gyro.y - gyroY_offset) * 180 / PI;
    Pitch = kalmanUpdate(&kalmanY, accAngle, gyroRate, dt);

    if (abs(Pitch) > 40) {
      stopMotors();
      Integral = 0;
    } else {
      // --- INTEGRATED COG COMPENSATION ---
      // This calculates the tilt needed to offset the arm's weight.
      // If the bot still drifts FORWARD when arm is UP, change -1.8 to a larger negative like -2.5
      float cogAdjustment = map(armCurrent, 53, 100, 1.8, -1.8); 
      
      float target = Neutral_Angle + Drive_Offset + cogAdjustment;
      Error = target - Pitch;

      // Anti-Windup Integral logic
      Integral = constrain(Integral + (Error * dt), -40, 40);
      
      float Derivative = (Error - PrevError) / dt;
      PrevError = Error;

      float PID = (Kp * Error) + (Ki * (Integral + Integral_Offset)) + (Kd * Derivative);

      int motorPWM = (int)PID;
      if (motorPWM > 0) motorPWM += Min_Speed;
      if (motorPWM < 0) motorPWM -= Min_Speed;
      
      driveMotors(motorPWM, Turn_Offset);
    }
  }
}
