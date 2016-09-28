#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define DECLINATION 7.50273 // Declination (degrees) in Detroit, MI.

void Accelerometer_Setup() {
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void Accelerometer_Loop() {
  
}

String getAccel() {
  imu.readAccel();
  return "ACCEL:" + (String)imu.calcAccel(imu.ax) + "," + (String)imu.calcAccel(imu.ay) + "," + (String)imu.calcAccel(imu.az) + "";
}

