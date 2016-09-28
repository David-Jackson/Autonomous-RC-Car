#include "Motor.h"
#include "Radio.h"

void setup() {
  Serial.begin(115200);
  printf_begin();
  Motor_Setup();
  Radio_Setup();
}

void loop() {
  Motor_Loop();
  Radio_Loop();
  Accelerometer_Loop();
}
