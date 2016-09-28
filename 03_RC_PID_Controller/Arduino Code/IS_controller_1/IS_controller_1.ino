#include<Servo.h>
#include<PID_v1.h>
#include "Motor.h"
#include<NewPing.h>

#define TRIGGER_PIN  5                                                //Trigger Pin for UltraSonic Sensor
#define ECHO_PIN 6                                                    //Echo Pin for UltraSonic Sensor
#define MAX_DISTANCE 30                                               //Max Distance to be sensed (cm)
const int servoPin = 9;                                               //Servo Pin
int potPin = A0;                                                      //Analog Potentiometer Pin 


float Kp = 0.8;                                                       //Initial Proportional Gain
float Ki = 0.07;                                                      //Initial Integral Gain
float Kd = 0.12;                                                      //Intitial Derivative Gain
double Setpoint, Input, Output;                                       

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);                  //Initialize NewPing object, which is in the class 
                                                                     //  NewPing that calcualtes and digitally filters 
                                                                     //  Ultrasonic Sensor data

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
                                                                     //  This class 'filters' the error signal based on 
                                                                     //  where the Setpoint and Input is. The Output is
                                                                     //  sent to the Servo in terms of degrees.
                                                                     
Servo myServo;                                                       //Initial Servo.

Motor drive;

void setup() {

  Serial.begin(9600);                                                //Begin Serial (to show readings)
  drive.init(3, 2, 4);

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  Setpoint = 300;                                                    //Desired location of ball in terms of potentiometer redading
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC (see playground for more details)
  myPID.SetOutputLimits(-150, 150);                                    //Set Output limits to 80 and 100 degrees. This keeps my servo from 
                                                                     //  going too far each way and messing up the plant.
}

void loop()
{
 
  Setpoint = 300;                                                    //Setpoint same as above in function setup() (see above for more details)
  Input = readPosition();                                            //Input same as above in function setup() (see above for more details)
  myPID.Compute();                                                   //The 'magic' happens and algorithm computes Output in range of 80 to 100 degrees
  drive.setMotor(Output);
  
  printData(Setpoint, Input, Output);                                //Calls function printData(). Prints Setpoint, Input, and Output to Serial    
  newGains();                                                        //Calls function newGains(). Changes Kp, Ki, and Kd and prints to Serial
}

//**********************************************************************************************************************************************************
//PrintData() prints Setpoint, Input, and Output to Serial each time PID loop runs. This may cause PID loop to run a tad slower than wanted
//            so it may be helpful to use this only when testing
//**********************************************************************************************************************************************************
void printData(float set, float in, float out) {
  Serial.print("Setpoint = "); 
  Serial.print(set); 
  Serial.print(" Input = "); 
  Serial.print(in); 
  Serial.print(" Output = "); 
  Serial.print(out); 
  Serial.print("\n");
}

//**********************************************************************************************************************************************************
//newGains() watis to see if anything is typed in serial port. If there is, the values are put into Kp, Ki, and Kd. Then the values are printed to Serial.
//**********************************************************************************************************************************************************
void newGains() {
  if (Serial.available() > 0) {                                        //If there is any data in serial input, then it starts reading.
     delay(100);                                                       //Pause for a tenth of a second to readjust plant or anything else you need to do.
                                                                       //  Can be as long as needed.     
     for (int i = 0; i < 4; i = i + 1) {                               
       switch (i) { 
         case 0:                                                       //Reads 1st value in
           Kp = Serial.parseFloat(); 
           break; 
         case 1:                                                       //Reads 2st value in 
           Ki = Serial.parseFloat(); 
           break; 
         case 2:                                                       //Reads 3st value in
           Kd = Serial.parseFloat(); 
           break; 
         case 3:                                                       //Clears any remaining parts.
           for (int j = Serial.available(); j == 0; j = j - 1) { 
             Serial.read();  
           }        
           break; 
       }
     }
     Serial.print(" Kp, Ki, Kd = ");                                     //Prints new gain values to Serial
     Serial.print(Kp); 
     Serial.print(", "); 
     Serial.print(Ki); 
     Serial.print(", "); 
     Serial.println(Kd);  
     myPID.SetTunings(Kp, Ki, Kd);
   }
   
}

//**********************************************************************************************************************************************************
//readPosition() reads the position of the ball and returns balls position in cm. 
//**********************************************************************************************************************************************************

float readPosition() {
  return analogRead(A1);
}



