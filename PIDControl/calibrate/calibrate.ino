#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// Declaration of pins, sensorData and counter
const int sensorPin = A1;
int sensorData1 = 0;        
int sensorData2 = 0;
int counter = 0;

void setup() {
  // initialize serial communications at 9600 bps
  Serial.begin(9600);
  // Signals communication with shield and motor
  AFMS.begin();
  // Low speed set up to track changes  
  myMotor->setSpeed(60);
  myMotor->run(FORWARD);
}

void loop() {
  // 50 tracking data
  while(counter<50){
    // Record data and delay 
    sensorData1 = analogRead(sensorPin);
    Serial.println(sensorData1);
    delay(50);
    // Increment counter
    counter++;
  }
  // Set speed of motor to zero
  myMotor->setSpeed(0);
}
