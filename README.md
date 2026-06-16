# Self-Balancing-Robot-Arm
A Bluetooth-controlled, two-wheeled self-balancing robot featuring an integrated robotic arm and dynamic Center of Gravity (COG) compensation
# ⚖️ Self-Balancing Robot with Robotic Arm

A Bluetooth-controlled, two-wheeled self-balancing robot featuring an integrated 2-DOF robotic arm. This project utilizes an MPU6050 IMU with a custom Kalman filter to maintain an upright position via PID control. 

**🌟 Key Feature:** The control loop includes dynamic Center of Gravity (COG) compensation. As the servo arm moves up and down, the robot automatically adjusts its neutral balancing angle to counteract the shifting weight, preventing the robot from drifting forward or falling.

## 🛠️ Hardware Requirements
* Arduino (Uno/Nano)
* MPU6050 Accelerometer & Gyroscope
* L298N Motor Driver
* 2x DC Motors (High RPM/Torque recommended)
* 2x Servo Motors (Arm and Gripper)
* HC-05 or HC-06 Bluetooth Module
* 11.1V LiPo Battery (recommended for motor torque)

## 📚 Required Libraries
* `Wire.h` (Built-in)
* `SoftwareSerial.h` (Built-in)
* `Servo.h` (Built-in)
* `Adafruit_MPU6050.h`
* `Adafruit_Sensor.h`

## ⚡ Wiring Guide

### MPU6050 (I2C)
* **VCC:** 5V
* **GND:** GND




* **SDA:** A4
* **SCL:** A5

### Bluetooth Module (HC-05/06)
* **VCC:** 5V
* **GND:** GND
* **TX:** D2 (Software RX)
* **RX:** D3 (Software TX via voltage divider)

### Servos
* **Arm Servo Signal:** D11
* **Grip Servo Signal:** D10
* *(Note: Power servos from a dedicated 5V buck converter, not the Arduino 5V pin, to prevent brownouts).*

### L298N Motor Driver
* **ENA:** D6 (PWM)
* **IN1:** A2
* **IN2:** A3
* **ENB:** D5 (PWM)
* **IN3:** D9
* **IN4:** D4

## 📱 Bluetooth Control Protocol
Send the following single-character commands via a Bluetooth terminal app to control the robot:

| Command | Action |
| :---: | :--- |
| **F** | Drive Forward |
| **B** | Drive Backward |
| **L** | Turn Left |
| **R** | Turn Right |
| **S** | Stop (Return to stationary balance) |
| **U** | Move Arm Up |
| **D** | Move Arm Down |
| **G** | Close Gripper |
| **O** | Open Gripper |

## ⚙️ Tuning the PID
If replicating this build, you will likely need to tune the PID and balancing variables for your specific frame weight and motor RPM. Adjust the following variables at the top of the sketch:
* `Neutral_Angle`: The resting angle where your specific bot is perfectly balanced.
* `Kp`, `Ki`, `Kd`: Standard PID tuning constants.
* `cogAdjustment`: Modify the `map()` values to change how much the bot compensates for the arm's weight shift.



  Play video for more to see working model :https://github.com/user-attachments/assets/1587eacc-256a-4a1b-9a82-d6259c7825c6
  https://youtu.be/_qBJa5aDV38?si=rlwt7iTYGCNxEU0e
  
