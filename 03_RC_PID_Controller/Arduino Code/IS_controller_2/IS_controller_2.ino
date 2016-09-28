
// Adafruit Motor Shield Libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// PID Library
#include<PID_v1.h>


// define IR sensor Pins
int STRAIGHT_PIN = 0; // analog pin for straight forward IR sensor
int RIGHT_PIN = 1; // analog pin for right IR sensor
int LEFT_PIN = 2; // analog pin for left IR sensor

// define constants
int TURN_THRESHOLD = 600; // side sensor reading when car will turn
boolean stopped = false; // variable to tell when car is stopped/should stop



// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1 is the drive and M4 is the steering
Adafruit_DCMotor *drive = AFMS.getMotor(1);
Adafruit_DCMotor *steer = AFMS.getMotor(4);

float Kp_drive = 0.8;                                                       //Initial Proportional Gain
float Ki_drive = 0.07;                                                      //Initial Integral Gain
float Kd_drive = 0.12;                                                      //Intitial Derivative Gain
double Setpoint_drive, Input_drive, Output_drive;                                       

PID drivePID(&Input_drive, &Output_drive, &Setpoint_drive, Kp_drive, Ki_drive, Kd_drive, DIRECT);


float Kp_steer = 0.8;
float Ki_steer = 0.07;
float Kd_steer = 0.12; 
double Setpoint_steer, Input_steer, Output_steer;                                       


PID steerPID(&Input_steer, &Output_steer, &Setpoint_steer, Kp_steer, Ki_steer, Kd_steer, DIRECT);


void setup() {

  Serial.begin(9600);

  
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Set drive speed to start, from 0 (off) to 255 (max speed)
  drive->setSpeed(0);
  drive->run(FORWARD);
  // turn on motor
  drive->run(RELEASE);
  // Set steer speed to start, from 0 (off) to 255 (max speed)
  steer->setSpeed(0);
  steer->run(FORWARD);
  // turn on motor
  steer->run(RELEASE);

  
  Input_drive = readPosition();
  Setpoint_drive = 300;
  drivePID.SetMode(AUTOMATIC);
  drivePID.SetOutputLimits(-50, 50);


  Input_steer = readPosition();
  Setpoint_steer = 650;
  steerPID.SetMode(AUTOMATIC);
  steerPID.SetOutputLimits(-40, 40);
  
}

long prevMillis = 0;

void loop()
{

  
  float right = readRight();
  float left = readLeft();
  stopped = false;
  
  if (right > TURN_THRESHOLD && left > TURN_THRESHOLD) {
    // stop
    Serial.print("STOPPED");
    stopped = true;
  } else if (right > TURN_THRESHOLD) {
    // turn left
    Serial.print("Left");
    Setpoint_steer = 500;
  } else if (left > TURN_THRESHOLD) {
    // turn right
    Serial.print("right");
    Setpoint_steer = 900;
  } else {
    Setpoint_steer = 700;
    Serial.print("straight");
  }

  Serial.print(" - ");
  Serial.println(readSteering());

  
  // calculate Steering PID Response
//  if (millis() - prevMillis > 5000) {
//    Setpoint_steer = 550;
//  }
//  if (millis() - prevMillis > 10000) {
//    prevMillis = millis();
//    Setpoint_steer = 900;
//  }
  Input_steer = readSteering();
  steerPID.Compute();
  setSteerSpeed(Output_steer);

  //printData(Setpoint_steer, Input_steer, Output_steer);
  
  // calculate Drive PID Response
  Setpoint_drive = 300;
  Input_drive = readPosition();
  drivePID.Compute();
  if (stopped) {
    setDriveSpeed(0);
  } else {
    setDriveSpeed(Output_drive);
  }
  delay(10);
}


void setDriveSpeed(int spd) {
  if (spd < 0) {
    // set to reverse
    drive->run(BACKWARD);
    drive->setSpeed(-spd);
  } else {
    // set to forward
    drive->run(FORWARD);
    drive->setSpeed(spd);
  }
}

void setSteerSpeed(int spd) {
  if (spd < 0) {
    // set to reverse
    steer->run(BACKWARD);
    steer->setSpeed(-spd);
  } else {
    // set to forward
    steer->run(FORWARD);
    steer->setSpeed(spd);
  }
}


void printData(float set, float in, float out) {
  Serial.print("Setpoint = "); 
  Serial.print(set); 
  Serial.print(" Input = "); 
  Serial.print(in); 
  Serial.print(" Output = "); 
  Serial.print(out); 
  Serial.print("\n");
}
float readPosition() {
  return analogRead(STRAIGHT_PIN);
}

float readSteering() {
  return analogRead(A5);
}

float readRight() {
  return analogRead(RIGHT_PIN);
}

float readLeft() {
  return analogRead(LEFT_PIN);
}



