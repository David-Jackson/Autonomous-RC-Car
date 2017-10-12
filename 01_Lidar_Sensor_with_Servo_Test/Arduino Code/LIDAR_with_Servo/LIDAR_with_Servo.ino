// THIS CODE USES A LIDAR SENSOR AND A SERVO TO SWEEP THE 90 DEGREES BACK 
// AND FORTH AND MEASURE THE DISTANCE AT DIFFERENT RATES TO DETERMINE WHICH 
// RATE IS MOST ACCURATE

#include <Servo.h>

Servo myservo;

int pos = 0;    // variable to store the servo position
int spd = 50;   // delay time in between movements
unsigned long pulse_width;

int passes = 1;

void setup() {
  myservo.attach(13);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); // Start serial communications
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
}

void loop() {
  if (passes < 10) {
  //myservo.write(0);
  for (pos = 45; pos <= 135; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(spd);                       // waits 15ms for the servo to reach the position
    checkDist();
    Serial.print(spd);
    Serial.print("\t");
    Serial.print(pos);
    Serial.print("\t");
    Serial.println(pulse_width);
  }
  for (pos = 135; pos >= 45; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(spd);                       // waits 15ms for the servo to reach the position
    checkDist();
    Serial.print(spd);
    Serial.print("\t");
    Serial.print(pos);
    Serial.print("\t");
    Serial.println(pulse_width);
  }
  passes++;
  } else {
    if (spd > 5) {
      spd = spd - 5;
      passes = 1;
    }
  }
}

void checkDist() {

  pulse_width = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  if(pulse_width != 0){ // If we get a reading that isn't zero, let's print it
        pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
    //Serial.println(pulse_width); // Print the distance
  }
  
}

