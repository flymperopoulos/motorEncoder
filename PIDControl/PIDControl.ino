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
const int tolerance = 200; 
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

// The chronicle of time
unsigned long time = 0;
int pastTime = 0;
int timer = 40;
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
  while(error > 0){
    time = millis();      
    sensorData1 = analogRead(sensorPin); 
    // If absolute variation is greater than the "threshold", then change and decrese the error     
    if(abs(sensorData2-sensorData1)>tolerance){
      // Decrease error      
      error--;
      delay(timer);
    }
    // Update integralState with new error term
    integralState += error*dt;    
    // Speed update
    maxSpeed = sectorAngle*kP*error + kI*integralState + 50;
    
    // Testing boundary speed state
    if(maxSpeed>255){
      myMotor->setSpeed(255);
    }
    else{
      myMotor->setSpeed(maxSpeed);
    }
    
    // Prints to console results
    Serial.print(millis());
    Serial.print(",");
    Serial.print(10-error);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.println(maxSpeed);
    
    // Updates sensorData
    sensorData2 = sensorData1;
    // Update time
    pastTime = time;
  }
  
  // Negative error case  
  while(error < 0){
    time = millis();
    myMotor->run(BACKWARD);
    sensorData1 = analogRead(sensorPin); 
    delay(2);
    sensorData2 = analogRead(sensorPin);
    // Checks for difference margin in more detail this time
    if(abs(sensorData2-sensorData1)>0.5*tolerance){
      // Increment error to revese direction
      error++; 
      delay(50);
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
    Serial.print(millis());
    Serial.print(",");
    Serial.print(10-error);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.println(maxSpeed);
  }

  // No-movement stage
  myMotor->run(RELEASE);
  // Speed set to zero
  myMotor->setSpeed(0);
  // Loops for error equal to zero
  while(error == 0){
    // Monitor sensor values
    sensorData1 = analogRead(sensorPin); 
    delay(2);
    sensorData2 = analogRead(sensorPin);
    //Sets error to -1 to restart
    if(abs(sensorData2-sensorData1)>tolerance){
      error = -1;
    }

  }
  delay(150);                     
}


