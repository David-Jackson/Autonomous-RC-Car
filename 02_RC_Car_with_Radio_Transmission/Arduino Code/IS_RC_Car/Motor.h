int enablePin = 3;
int in1Pin = 2;
int in2Pin = 4;


int index = 0;
unsigned long completedTime = 0;
int instrLength = 4;
int instr[4][2] = {
  {-255, 2000},
  {0, 5000},
  {255, 2000},
  {0, 5000}
};

void check(unsigned long ms) {
  if(ms - completedTime > instr[index][1]) {
    completedTime += instr[index][1];
    index = (index + 1) % instrLength;
  }
  //setMotor(instr[index][0]);
}

void setMotor(int spd) {
  boolean rev = spd >= 0;
  analogWrite(enablePin, abs(spd));
  digitalWrite(in1Pin, !rev);
  digitalWrite(in2Pin, rev);
}

void Motor_Setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
}

void Motor_Loop() {
  check(millis());
}



