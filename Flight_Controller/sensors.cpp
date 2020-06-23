#include <Wire.h>
#include "Arduino.h"
#include "config.h"
#include "defines.h"
#include "check.h"
#include "sensors.h"

#ifdef MPU6050
byte sensorSetup() {
  byte error;
  Wire.begin();
  Wire.setClock(400000);
  PORTC &= B11001111;   //disable internal pullups
  Wire.beginTransmission(0x68);
  Wire.write(107);   //register number
  Wire.write(0);    //disable sleep mode
  error = Wire.endTransmission();
  if (error > 0) {
    return error;
  }
  Wire.beginTransmission(0x68);
  Wire.write(25);      //register number
  Wire.write(1);       // sample rate divider (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) = 1000/(1+1))
  Wire.write(LOW_PASS_FILTER);  //enable digital low pass filter 
  Wire.write(B00011000);   //gyro settings
  Wire.write(B00010000);   // acc settings
  error = Wire.endTransmission();
  return error;
}

void getGyro() {
  Wire.beginTransmission(0x68);
  Wire.write(67);      //register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  while (Wire.available() < 5) {}
  gyroData[0] = Wire.read() << 8;
  gyroData[0] |= Wire.read();
  gyroData[1] = Wire.read() << 8;
  gyroData[1] |= Wire.read();
  gyroData[2] = Wire.read() << 8;
  gyroData[2] |= Wire.read();
  Wire.endTransmission();
  gyroData[GYRO_X] *= GYRO_X_INVERT;
  gyroData[GYRO_Z] *= GYRO_Z_INVERT;
  gyroData[GYRO_Y] *= GYRO_Y_INVERT;
}

void getAcc() {

  Wire.beginTransmission(0x68);
  Wire.write(59);      //register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  while (Wire.available() < 5) {}
  accData[0] = Wire.read() << 8;
  accData[0] |= Wire.read();
  accData[1] = Wire.read() << 8;
  accData[1] |= Wire.read();
  accData[2] = Wire.read() << 8;
  accData[2] |= Wire.read();
  Wire.endTransmission();
  accData[ACC_X] *= ACC_Y_INVERT;    // x-axis acceleration is used to calculate orientation on the y-axis
  accData[ACC_Y] *= ACC_X_INVERT;
  accData[ACC_Z] *= ACC_Z_INVERT;
}
#endif
