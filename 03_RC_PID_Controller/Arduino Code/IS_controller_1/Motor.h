class Motor {
  public:
    int enablePin;
    int in1Pin;
    int in2Pin;  
    
  
    void init(int EP, int in1, int in2) {
      enablePin = EP;
      in1Pin = in1;
      in2Pin = in2;
      pinMode(enablePin, OUTPUT);
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);
    };
  
    void setMotor(int spd) {
      boolean rev = spd >= 0;
      analogWrite(enablePin, abs(spd));
      digitalWrite(in1Pin, !rev);
      digitalWrite(in2Pin, rev);
    };
    void setMotor(int spd, bool rev) {
      analogWrite(enablePin, spd);
      digitalWrite(in1Pin, !rev);
      digitalWrite(in2Pin, rev);
    };
    
  
};
