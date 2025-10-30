#include "arduino_secrets.h"

#include <Wire.h> 
#include <SPI.h> 
#include <Adafruit_LSM9DS1.h> 
#include <Adafruit_Sensor.h> 
#include <SoftwareSerial.h> 
 
// LSM9DS1 pins 
#define LSM9DS1_SCK A5 
#define LSM9DS1_MOSI A4 
 
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); 
 
SoftwareSerial BTSerial(0, 1); // RX, TX 
 
// Motor and Encoder pins 
#define motorA 5 
#define motorB 6 
int encoderPin1 = 2; 
int encoderPin2 = 3; 
 
volatile int lastEncoded = 0; 
volatile long encoderValue = 0; 
volatile long correctEncoderValue = 0; 
volatile long lastencoderValue = 0; 
int lastMSB = 0; 
int lastLSB = 0; 
int targetSpeed = 0; 
 
unsigned long lastTime = 0; // Variable to store the last time 
the RPM was calculated 
unsigned long lastTime1 = 0; // Variable to store the last 
time data was printed in the serial monitor 
float rpm = 0; // Variable to store the calculated RPM 
const int CPR = 64; // Counts per revolution of the encoder 
 
float targetGyroZ = 0; // Target Gyro Z value in deg/s 
float currentGyroZ = 0; // Current Gyro Z value in deg/s 
float currentAccelZ = 0; 
float Kp = 0.5; // Proportional gain for the controller 
 
float error = 0; // Variable to sstore the calculated error 
int adjustment = 0; // Variable to store the calculated 
adjustment to targetSpeed 
int adjustmentStep = 10; // Variable to control max abs value 
of adjustment 
 
void setup() 
{ 
 Serial.begin(115200); 
 while (!Serial) 
 { 
  delay(1); // wait for Serial to connect 
 } 
 BTSerial.begin(115200); 
 while (!BTSerial) 
 { 
  delay(1); // wait for Serial to connect 
 } 
 Wire.begin(); 
 if (!lsm.begin()) 
 { 
  Serial.println("Unable to initialize the 
LSM9DS1 9DOF"); 
  while (1); 
 } 
 Serial.println("Found LSM9DS1 9DOF"); 
 
 // Initialize sensor settings 
 setupSensor(); 
 
 // Initialize motor and encoder pins 
 pinMode(encoderPin1, INPUT); 
 pinMode(encoderPin2, INPUT); 
 digitalWrite(encoderPin1, HIGH); // Turn pull-up 
resistor on 
 digitalWrite(encoderPin2, HIGH); // Turn pull-up 
resistor on 
 attachInterrupt(digitalPinToInterrupt(encoderPin1), 
updateEncoder, CHANGE); 
 attachInterrupt(digitalPinToInterrupt(encoderPin2), 
updateEncoder, CHANGE); 
 
 pinMode(motorA, OUTPUT); 
 pinMode(motorB, OUTPUT); 
} 
 
void loop() 
{ 
 // Read sensor data 
 sensors_event_t accel, mag, gyro, temp; 
 lsm.getEvent(&accel, &mag, &gyro, &temp); 
 currentGyroZ = gyro.gyro.z * 57.29578; // Convert 
to deg/s 
 currentAccelZ = accel.acceleration.z; 
 
 // Calculate RPM every second 
 correctEncoderValue = encoderValue / 4; 
 if (millis() - lastTime >= 1000) 
 { 
  noInterrupts(); // Disable interrupts while 
calculating RPM 
  rpm = (abs(correctEncoderValue) / 
(float)CPR) * 60.0; 
  encoderValue = 0; 
  correctEncoderValue = 0; // Reset the 
counter 
  lastTime = millis(); // Update the last time 
  interrupts(); // Re-enable interrupts 
 } 
 
 if (millis() - lastTime1 >= 1000) 
 { 
  // Print sensor data 
  Serial.print("Accel Z: "); 
Serial.print(currentAccelZ); Serial.print(" m/s^2"); 
  Serial.print("Gyro Z: "); 
Serial.print(currentGyroZ); Serial.println(" deg/s"); 
 
  // Print RPM and encoder count to serial 
monitor 
  Serial.print("Encoder Count: "); 
  Serial.println(correctEncoderValue); 
  Serial.print("RPM: "); 
  Serial.println(rpm); 
  Serial.println(); 
  lastTime1 = millis(); // Update the last 
time 
 } 
 
 // Read desired Gyro Z value from serial monitor 
 if (Serial.available()) 
 { 
  targetGyroZ = Serial.parseFloat(); 
  Serial.print("Target Gyro Z = "); 
  Serial.print(targetGyroZ); 
  Serial.println(" deg/s"); 
  Serial.println(); 
 } 
 
 // Proportional control to adjust motor speed 
 error = targetGyroZ - currentGyroZ; 
 adjustment = (int)(Kp * error); 
 
 // Constrain adjustment to prevent sudden changes 
in speed 
 adjustment = constrain(adjustment, 
adjustmentStep, adjustmentStep); 
 
 targetSpeed += adjustment; 
 
 // Constrain targetSpeed to valid PWM range 
 targetSpeed = constrain(targetSpeed, -255, 255); 
 
 Serial.print("Target Speed = "); 
 Serial.println(targetSpeed); 
 
 // Control motor speed based on targetSpeed 
 if(targetSpeed >= 0) 
 { 
  analogWrite(motorA, targetSpeed); 
  digitalWrite(motorB, HIGH); 
 } 
 else 
 { 
  analogWrite(motorA, -targetSpeed); 
  digitalWrite(motorB, LOW); 
 } 
 
 delay(100); 
} 
 
void setupSensor() 
{ 
 // Set the accelerometer range 
 lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2
 G, lsm.LSM9DS1_ACCELDATARATE_10HZ); 
 
 // Set the magnetometer sensitivity 
 lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAU
 SS); 
 
 // Setup the gyroscope 
 lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245
 DPS); 
} 
 
void updateEncoder() 
{ 
 int MSB = digitalRead(encoderPin1); // MSB = 
most significant bit 
 int LSB = digitalRead(encoderPin2); // LSB = least 
significant bit 
 
 int encoded = (MSB << 1) | LSB; // Converting the 
2 pin value to a single number 
 int sum = (lastEncoded << 2) | encoded; // Adding 
it to the previous encoded value 
 
 if (sum == 0b1101 || sum == 0b0100 || sum == 
0b0010 || sum == 0b1011) encoderValue++; 
 if (sum == 0b1110 || sum == 0b0111 || sum == 
0b0001 || sum == 0b1000) encoderValue--; 
 
 correctEncoderValue += (sum == 0b1101 || sum == 
0b0100 || sum == 0b0010 || sum == 0b1011) ? 1 : -1; 
 
 if (correctEncoderValue >= CPR || 
correctEncoderValue <= -CPR) 
 { 
  correctEncoderValue = 0; // Reset after a 
full rotation 
 } 
 
 lastEncoded = encoded; // Store this value for next 
time 
}