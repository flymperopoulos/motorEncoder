#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor = AFMS.getMotor(1);

// Declaration of the sensor pin
const int sensorPin = A1;
const int tolerance = 10;

// Angle of sections
const int angle = 9;

// Reads data from sensor
float sensorData1;
float sensorData2;
float kP;
float kI;

int sumErrors = 0;
// Variables for data recording without delay()
long previousMillis = 0;
long intervalWait = 3;

// Initiates error and counter
int error = 90;
int counter = 0;

void setup() {
  // Serial communication enabled  
  Serial.begin(9600);
  AFMS.begin();
  
  motor->setSpeed(160);
  motor->run(FORWARD);
  
  kP = 200/180.0;
  kI = 100;
}

void loop() {
  sensorData1 = analogRead(sensorPin);
  positiveError();

  noError();
  negativeError();
  delay(10000);
}

void positiveError(){
  motor->run(FORWARD);
  
  while (error>0){
    unsigned long currentMillis = millis();
        
    if (currentMillis - previousMillis > intervalWait) {
      previousMillis = currentMillis;
      sensorData2 = analogRead(sensorPin);
    }
      
    if (sensorData2 < sensorData1 +tolerance || sensorData1 > sensorData2 - tolerance) {
      error = error - angle;
      Serial.println(sensorData1);
      Serial.println(sensorData2);
      sensorData1 = sensorData2;
      counter ++;
      sumErrors+=(error*intervalWait);
      motor->setSpeed(kP*error+kI*sumErrors);
    }
    printError();
    delay(1000);
  }
}

void noError(){
  motor->run(RELEASE);
  while (error == 0){
    
    motor->setSpeed(kP*error);
    
    printError();
  }
}

void negativeError(){
  motor->run(BACKWARD);
  
  while (error<0){
    unsigned long currentMillis = millis();
    
    sensorData1 = analogRead(sensorPin);
    
    if (currentMillis - previousMillis > intervalWait) {
      previousMillis = currentMillis;
      sensorData2 = analogRead(sensorPin);
    }
      
    if (sensorData1 < sensorData2 - tolerance || sensorData1 > sensorData2 + tolerance) {
      error = error + angle;
      counter ++;
      sumErrors+=(error*intervalWait);
      motor->setSpeed(kP*error+kI*sumErrors);
    }
  }
  printError();
  delay(1000);
}

void printError(){
  Serial.println("The error is: ");
  Serial.println(error);
}
