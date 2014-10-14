#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// Declaration of Analog Pin for reflective sensor 
const int sensorPin = A1;  
// Declaration of data collecting points
int sensorData1 = 0;        
int sensorData2 = 0;
// Tolerance definition to eliminate reading noise
const int tolerance = 100; 
// Sector Angle definition
const int sectorAngle = 9; 
// Initiate an error as the number of arcs in the 90 degree pattern
int error = 10;
// Initial maximum speed of the motor
int maxSpeed = 0;
// Integral state defined initially zero
double integralState = 0; 

// Proportional and Integral control constants
double kI = 5; 
double kP = 1; 

// Input initial wave value
double inputWave = 0;
// Pi defintiion
const float pi = 3.141594;
// Position of motor
int getPosition = 0;

// Range of the values
const int range = 120;

// The chronicle of time
unsigned long time = 0;
int pastTime = 0;
int timer = 80;
float dt = 0;

void setup() {
  // initialize serial communications at 9600 bps
  Serial.begin(9600);
  // initializes motor with shield  
  AFMS.begin();
  // Initialize my motor  
  myMotor->run(FORWARD);
  myMotor->setSpeed(maxSpeed);
}

void loop() {
  // Read first analog sensor value  
  sensorData2 = analogRead(sensorPin);  

  // Loop for positive error (1-10)  
  while(inputWave >= range){
    time = millis();
    // Input wave
    inputWave = 120*(sin(time/1000*pi)+1);
    myMotor->run(FORWARD);  
    sensorData1 = analogRead(sensorPin); 
    // If absolute variation is greater than the "threshold", then change and decrese the error     
    if(abs(sensorData2-sensorData1)>tolerance){
      getPosition=getPosition + sectorAngle;
      // Decrease error      
      error = inputWave - getPosition;
      delay(timer);
    }
    // Update integralState with new error term
    integralState += error*dt;    
    pastTime = time;
    // Speed update
    maxSpeed = sectorAngle*kP*error + kI*integralState;
    
    // Testing boundary speed state
    if(maxSpeed>255){
      myMotor->setSpeed(255);
    }
    else{
      myMotor->setSpeed(maxSpeed);
    }
    
    // Prints to console results
    Serial.print(time);
    Serial.print(",");
    Serial.print(inputWave);
    Serial.print(",");
    Serial.print(getPosition);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.println(maxSpeed);
    
    // Updates sensorData
    sensorData2 = sensorData1;
    // Update time
  }
  
  // Negative error case  
  while(inputWave < range){
    time = millis();
    inputWave = 120*(sin(time/1000)+1);
    myMotor->run(BACKWARD);
    sensorData1 = analogRead(sensorPin); 
    // Checks for difference margin in more detail this time
    if(abs(sensorData2-sensorData1)>0.5*tolerance){
      // Increment error to revese direction
      getPosition=getPosition - sectorAngle;
      // Decrease error      
      error = inputWave - getPosition;
      delay(timer);
    }
    // Updates integralState
    integralState += error*dt;
    // Re-assignment of time
    pastTime = time;
    // Update of motor speed
    maxSpeed = abs(sectorAngle*kP*error + kI*integralState + 50);
    // Checks for boundaries
    if(maxSpeed>255){
      myMotor->setSpeed(255);
    }
    else{
      myMotor->setSpeed(maxSpeed);
    }
    
    // Prints to console results
    Serial.print(time);
    Serial.print(",");
    Serial.print(inputWave);
    Serial.print(",");
    Serial.print(getPosition);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.println(maxSpeed);
  }
  delay(100);                     
}


