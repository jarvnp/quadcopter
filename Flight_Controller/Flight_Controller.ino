#include <Wire.h>
#include "config.h"
#include "defines.h"
#include "check.h"
#include "sensors.h"

#ifdef USE_EEPROM
#include <EEPROM.h>
#endif


volatile unsigned long t1Ovf = 0;
unsigned int outputVals[OUTPUT_AMOUNT];
byte order[OUTPUT_AMOUNT];
byte smallest = 0;
byte biggest = 0;
byte temp;
unsigned long outputTimes[OUTPUT_AMOUNT];

volatile unsigned int inputVals[CHANNEL_AMOUNT + 1]; //one extra for the delay between channels
byte channel = 255;
unsigned long signalTimer1 = 0;
unsigned long signalTimer2 = 0;
bool dataLevel;
volatile bool signalFound = 0;
volatile bool updated = 0;
unsigned long updateTimer = 0;
unsigned long checkTimer = 0;
bool signalIsGood;
volatile bool inputValsUpdated = false;
unsigned int savedInputVals[CHANNEL_AMOUNT + 1];
unsigned long inputValsUpdatedTimer;
float expo[3] = {EXPO_PITCH, EXPO_ROLL, EXPO_YAW};

bool signalLostTimerSet = false;
unsigned long signalLostTimer;

int gyroData[3] = {0, 0, 0};
int prevGyroData[3] = {0, 0, 0};
int accData[3];
int prevAccData[3] = {0, 0, 0};

#ifdef AUTO_LEVEL_MODE
int sinGyroZ;
byte accFact = 32;
unsigned long hyp;
long accPosX = 0;
long accPosY = 0;
#endif

long posOffset[2] = {0, 0};
long pos[3] = {0, 0, 0};
int gyroOffset[3];
unsigned long startTime;
long loopTime = 0;
long temp2;
unsigned long normalHyp;
const byte gyroXYZ[] = {GYRO_X, GYRO_Y, GYRO_Z};
const byte accXYZ[] = {ACC_X, ACC_Y, ACC_Z};
long tempGyroOffset[3] = {0, 0, 0};
unsigned int gyroCalibCounter = 0;
bool gyroCalib = true;
bool accCalib = true;
unsigned int accCalibCounter = 0;
long accAvg[3] = {0, 0, 0};
unsigned int maxAngularRatePitch = MAX_ANGULAR_RATE_PITCH;
unsigned int maxAngularRateRoll = MAX_ANGULAR_RATE_ROLL;
unsigned int maxAngularRateYaw = MAX_ANGULAR_RATE_YAW;




int PID_P[3];   //X, Y, Z (PITCH, ROLL, YAW)
int PID_I[3];
int PID_D[3];
int error[3]; //X, Y, Z (PITCH, ROLL, YAW)
int lastError[3];
int correction[3];//XYZ
int lastCorrection[3];
int motorCorrection[4];
int throttleOffset = 0;

long pidKp[][3] = {{PID_Kp_PITCH_ACRO, PID_Kp_ROLL_ACRO, PID_Kp_YAW}
#ifdef AUTO_LEVEL_MODE
  , {PID_Kp_PITCH_AL, PID_Kp_ROLL_AL, PID_Kp_YAW}};
#else
};
#endif
long pidKi[][3] = {{PID_Ki_PITCH_ACRO, PID_Ki_ROLL_ACRO, PID_Ki_YAW}
#ifdef AUTO_LEVEL_MODE
  , {PID_Ki_PITCH_AL, PID_Ki_ROLL_AL, PID_Ki_YAW}};
#else
};
#endif
long pidKd[][3] = {{PID_Kd_PITCH_ACRO, PID_Kd_ROLL_ACRO, PID_Kd_YAW}
#ifdef AUTO_LEVEL_MODE
  , {PID_Kd_PITCH_AL, PID_Kd_ROLL_AL, PID_Kd_YAW}};
#else
};
#endif

bool armed = false;
unsigned int loopWaitTime = 20000;
unsigned long serialTimer = 0;
char data[20];
String value;
String command;

#ifdef USE_EEPROM
byte saveToEeprom = 0;
int dataToBeSaved[10];
int addressToBeSaved[10];
/*
bool PIDInEeprom[] = {false, false};
bool accCalibInEeprom = false;
bool gyroCalibInEeprom = false;
*/
const unsigned int pidStartAddresses[2] = {PID_VALUES_ACRO_START_ADDRESS, PID_VALUES_AL_START_ADDRESS};
//const unsigned int pidPWAddresses[2] = {PID_VALUES_ACRO_PW_ADDRESS, PID_VALUES_AL_PW_ADDRESS};
byte dataInEeprom;
const byte pidValueBits[2] = {ACRO_PID,AL_PID};
#endif

#ifdef BUZZER
unsigned int buzzer[3] = {0, 0, 0}; // {onTime(ms), offTime(ms), how_many_repeats}
bool buzzerOn = false;
unsigned long buzzerTimer = 0;
#endif

byte flightMode = 0; // 0: acro, 1: autolevel

unsigned long printTimer = 0;
bool buzzerAlarm = false;
unsigned long armTimer = 4294966295;



#if(EXTRA_OUTPUT_AMOUNT==4)
const byte outputPins[] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, EXTRA_OUTPUT_1_PIN, EXTRA_OUTPUT_2_PIN, EXTRA_OUTPUT_3_PIN, EXTRA_OUTPUT_4_PIN};
#endif
#if(EXTRA_OUTPUT_AMOUNT==3)
const byte outputPins[] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, EXTRA_OUTPUT_1_PIN, EXTRA_OUTPUT_2_PIN, EXTRA_OUTPUT_3_PIN};
#endif
#if(EXTRA_OUTPUT_AMOUNT==2)
const byte outputPins[] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, EXTRA_OUTPUT_1_PIN, EXTRA_OUTPUT_2_PIN};
#endif
#if(EXTRA_OUTPUT_AMOUNT==1)
const byte outputPins[] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, EXTRA_OUTPUT_1_PIN};
#endif
#if(EXTRA_OUTPUT_AMOUNT==0)
const byte outputPins[] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN};
#endif



void setup() {
  
  Serial.begin(BAUD_RATE);
  #ifdef BUZZER
  setAsOutput(BUZZER_PIN);
#endif
    noInterrupts();
  __asm__("wdr"); //set watchdog timer to reset the system if the loop for some reason stops running for 0.125 seconds
  WDTCSR |=B00011000;
  WDTCSR = B00001011;
  interrupts();

#ifdef BUZZER //if for example the sensor disconnects on crash, the program would never even get to the loop. In that case the watchdog would repeatedly reset the system and the buzzer would beep, which will make it easier to find your quad
  pinHigh(BUZZER_PIN);
  delay(50);
  __asm__("wdr"); 
  pinLow(BUZZER_PIN);
  delay(50);
  __asm__("wdr"); 
#endif
  
  while (sensorSetup() > 0) {
    Serial.println("Sensorsetup error");
  }
  /* delay(100);
    Serial.println("Calibrating gyro, don't move");
    while (gyroCalib() == false) {
    Serial.println("DONT' MOVE ME");
    }
    Serial.println("Done");
    delay(100);
    Serial.println("Calibrating acc, don't move");
    while (accCalib() == false) {
    Serial.println("DONT' MOVE ME");
    }
    normalHyp = (int)sqrt(((long)accAvg[0] * accAvg[0]) + ((long)accAvg[1] * accAvg[1]) + ((long)accAvg[2] * accAvg[2]));
    posOffset[1] = (long)(256 * (asin((float)accAvg[0] / normalHyp) * 57.29578));
    posOffset[0] = (long)(256 * (asin((float)accAvg[1] / normalHyp) * 57.29578));
    Serial.println("Done");*/
  for (byte i = 0; i < 3; i++) {
    pos[i] = 0;
  }
  for (byte i = 0; i < OUTPUT_AMOUNT; i++) {
    outputVals[i] = 900;
    setAsOutput(outputPins[i]);
  }


  INPUT_SETUP;


  TCCR1A &= B00001100;   //timer 1 setup prescaler 8
  TCCR1B &= B11100000;
  TCCR1B |= B00000010;
  TIMSK1 |= B00000001;




#if(EXTRA_OUTPUT_AMOUNT==4)
  setAsOutput(EXTRA_OUTPUT_1_PIN); setAsOutput(EXTRA_OUTPUT_2_PIN); setAsOutput(EXTRA_OUTPUT_3_PIN); setAsOutput(EXTRA_OUTPUT_4_PIN);
#endif
#if(EXTRA_OUTPUT_AMOUNT==3)
  setAsOutput(EXTRA_OUTPUT_1_PIN); setAsOutput(EXTRA_OUTPUT_2_PIN); setAsOutput(EXTRA_OUTPUT_3_PIN);
#endif
#if(EXTRA_OUTPUT_AMOUNT==2)
  setAsOutput(EXTRA_OUTPUT_1_PIN); setAsOutput(EXTRA_OUTPUT_2_PIN);
#endif
#if(EXTRA_OUTPUT_AMOUNT==1)
  setAsOutput(EXTRA_OUTPUT_1_PIN);
#endif



#ifdef USE_EEPROM
  if(EEPROM[EEPROM_CHECK_ADDRESS] != 123){    // 123 is the "password" that tells the program that there is useful data in eeprom.
    EEPROM[VALUES_IN_EEPROM_ADDRESS] = 0;
    EEPROM[EEPROM_CHECK_ADDRESS] = 123;
  }
  dataInEeprom = EEPROM[VALUES_IN_EEPROM_ADDRESS];
  if ((dataInEeprom >> ACRO_PID)& 1) { 
    for (byte i = 0; i < 3; i++) {
      pidKp[0][i] = eepromReadInt(i * 2 + PID_VALUES_ACRO_START_ADDRESS);
      pidKi[0][i] = eepromReadInt(i * 2 + 6 + PID_VALUES_ACRO_START_ADDRESS);
      pidKd[0][i] = eepromReadInt(i * 2 + 12 + PID_VALUES_ACRO_START_ADDRESS);
    }
  }
  #ifdef AUTO_LEVEL_MODE
  if ((dataInEeprom >> AL_PID)& 1) {
    for (byte i = 0; i < 3; i++) {
      pidKp[1][i] = eepromReadInt(i * 2 + PID_VALUES_AL_START_ADDRESS);
      pidKi[1][i] = eepromReadInt(i * 2 + 6 + PID_VALUES_AL_START_ADDRESS);
      pidKd[1][i] = eepromReadInt(i * 2 + 12 + PID_VALUES_AL_START_ADDRESS);
    }
  }
  #endif
  if ((dataInEeprom >> ACC_CALIB)& 1) { 
    accCalib = false;
    for (byte i = 0; i < 3; i++) {
      accAvg[i] = eepromReadInt(i * 2 + ACC_CALIB_START_ADDRESS);
    }
    normalHyp = (int)sqrt(((long)accAvg[0] * accAvg[0]) + ((long)accAvg[1] * accAvg[1]) + ((long)accAvg[2] * accAvg[2]));
    posOffset[1] = (long)(1000 * (asin((float)accAvg[0] / normalHyp) * 57.29578));
    posOffset[0] = (long)(1000 * (asin((float)accAvg[1] / normalHyp) * 57.29578));
  }
  if ((dataInEeprom >> GYRO_CALIB)& 1) { 
    gyroCalib = false;
    for (byte i = 0; i < 3; i++) {
      gyroOffset[i] = eepromReadInt(i * 2 + GYRO_CALIB_START_ADDRESS);
    }
  }
  if((dataInEeprom >> MAX_RATES)& 1){
    maxAngularRatePitch = eepromReadInt(MAX_RATES_START_ADDRESS);
    maxAngularRateRoll = eepromReadInt(MAX_RATES_START_ADDRESS+2);
    maxAngularRateYaw = eepromReadInt(MAX_RATES_START_ADDRESS+4);
  }
  if((dataInEeprom >> EXPO)& 1){
    for(uint8_t i=0; i<3; i++){
      expo[i] = (float)eepromReadInt(EXPO_START_ADDRESS+ i*2)/10000.0;
    }
  }
#endif
  startTime = microSeconds();
}

void loop() {
  __asm__("wdr"); //reset watchdog timer
  loopTime = (long)(microSeconds() - startTime);
  startTime = microSeconds();
  getAcc();
  getGyro();
  if (gyroCalib && (!armed)) {
    if (gyroCalibCounter < 400) {
      tempGyroOffset[0] += (long)gyroData[0];
      tempGyroOffset[1] += (long)gyroData[1];
      tempGyroOffset[2] += (long)gyroData[2];
      gyroCalibCounter++;
      if (gyroCalibCounter > 30) {
        for (byte i2 = 0; i2 < 3; i2++) {
          if (abs(gyroData[i2] - (tempGyroOffset[i2] / (gyroCalibCounter + 1))) > 100) {
            gyroCalibCounter = 0;
            for (byte i = 0; i < 3; i++) {
              tempGyroOffset[i] = 0;
            }
#ifdef BUZZER
            buzzerFunc(100, 100, 3);
#endif
            break;
          }
        }
      }
    }
    else {
#ifdef BUZZER
            buzzerFunc(50, 300, 2);
#endif
      //Serial.println("gyro done");
      for (byte i = 0; i < 3; i++) {
        gyroOffset[i] = (int)(tempGyroOffset[i] / 400);
       // Serial.println(gyroOffset[i]);
      }
      gyroCalibCounter = 0;
      gyroCalib = false;
      for (byte i = 0; i < 3; i++) {
        tempGyroOffset[i] = 0;
      }
    }
  }
  if (accCalib && (!armed)) {
    if (accCalibCounter < 400) {
      accAvg[0] += (long)accData[ACC_X];
      accAvg[1] += (long)accData[ACC_Y];
      accAvg[2] += (long)accData[ACC_Z];
      accCalibCounter++;
      if (accCalibCounter > 30) {
        for (byte i2 = 0; i2 < 3; i2++) {
          if (abs(accData[accXYZ[i2]] - (accAvg[i2] / (accCalibCounter + 1))) > 200) {
            accCalibCounter = 0;
            for (byte i = 0; i < 3; i++) {
              accAvg[i] = 0;
            }
#ifdef BUZZER
            buzzerFunc(100, 100, 3);
#endif
            break;
          }
        }
      }
    }
    else {
#ifdef BUZZER
            buzzerFunc(50, 300, 2);
#endif
      pos[0] = 0;
      pos[1] = 0;
     // Serial.println("acc done");
      accCalib = false;
      accCalibCounter = 0;
      for (byte i = 0; i < 3; i++) {
        accAvg[i] /= 400;
      //  Serial.println(accAvg[i]);
      }
      //normalHyp = (int)sqrt(((long)accAvg[0] * accAvg[0]) + ((long)accAvg[1] * accAvg[1]) + ((long)accAvg[2] * accAvg[2]));
      posOffset[1] = (long)(1000.0 * (atan((float)accAvg[0] / accAvg[2]) * 57.29578));
      posOffset[0] = (long)(1000.0 * (atan((float)accAvg[1] / accAvg[2]) * 57.29578));
    }
  }
  for (byte i = 0; i < 3; i++) {
    gyroData[i] -= gyroOffset[i];
  }
  for (byte i = 0; i < 3; i++) {
    gyroData[i] = (int)((((long)gyroData[i] * 128) + ((long)prevGyroData[i] * (256 - 128))) / 256);
    prevGyroData[i] = gyroData[i];
    accData[i] = (int)((((long)accData[i] * 32) + ((long)prevAccData[i] * (256 - 32))) / 256);
    prevAccData[i] = accData[i];
  }
  if (signalIsGood) {
    signalLostTimerSet = false;
    auxSwitches();
  }
  else {
    if (!signalLostTimerSet) {
      signalLostTimer = millis();
      signalLostTimerSet = true;
    }
    if ((signalLostTimer + 400) < millis()) {
      armed = false;
    }
  }


  if (armed == false) {
#ifdef USE_EEPROM
    if (saveToEeprom > 0) {
      eepromWriteInt(addressToBeSaved[saveToEeprom - 1], dataToBeSaved[saveToEeprom - 1]);
      saveToEeprom--;
    }
#endif
    if (Serial.available()) {
      if (serialTimer == 0) {
        serialTimer = millis();
      }
      else if (serialTimer + 200 < millis()) {
        value = "";
        command = "";
        byte serialDataCounter = 0;
        serialTimer = 0;
        while (Serial.available() && serialDataCounter < 20) {
          data[serialDataCounter] = Serial.read();
          serialDataCounter++;
        }
        byte index = 0;
        while ((data[index] != 32) && index < serialDataCounter) {
          command += data[index];
          index++;
        }
        index++;
        while (index < serialDataCounter) {
          value += data[index];
          index++;
        }
        if (command == COMMAND_P_PITCH) {
          pidKp[flightMode][0] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKp[flightMode][0]);
        }
        else if (command == COMMAND_P_ROLL) {
          pidKp[flightMode][1] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKp[flightMode][1]);
        }
        else if (command == COMMAND_P_YAW) {
          pidKp[flightMode][2] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKp[flightMode][2]);
        }
        else if (command == COMMAND_I_PITCH) {
          pidKi[flightMode][0] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKi[flightMode][0]);
        }
        else if (command == COMMAND_I_ROLL) {
          pidKi[flightMode][1] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKi[flightMode][1]);
        }
        else if (command == COMMAND_I_YAW) {
          pidKi[flightMode][2] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKi[flightMode][2]);
        }
        else if (command == COMMAND_D_PITCH) {
          pidKd[flightMode][0] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKd[flightMode][0]);
        }
        else if (command == COMMAND_D_ROLL) {
          pidKd[flightMode][1] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKd[flightMode][1]);
        }
        else if (command == COMMAND_D_YAW) {
          pidKd[flightMode][2] = atol(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(pidKd[flightMode][2]);
        }
        else if (command == COMMAND_PRINT_PID) {
          Serial.print("Pitch(P,I,D): ");
          Serial.print(pidKp[flightMode][0]);
          Serial.print(",");
          Serial.print(pidKi[flightMode][0]);
          Serial.print(",");
          Serial.println(pidKd[flightMode][0]);
          Serial.print("Roll(P,I,D): ");
          Serial.print(pidKp[flightMode][1]);
          Serial.print(",");
          Serial.print(pidKi[flightMode][1]);
          Serial.print(",");
          Serial.println(pidKd[flightMode][1]);
          Serial.print("Yaw(P,I,D): ");
          Serial.print(pidKp[flightMode][2]);
          Serial.print(",");
          Serial.print(pidKi[flightMode][2]);
          Serial.print(",");
          Serial.println(pidKd[flightMode][2]);
        }
        else if(command == COMMAND_MAX_RATE_PITCH){
          maxAngularRatePitch = value.toInt();
          Serial.print(command);
          Serial.print(" ");
          Serial.println(maxAngularRatePitch);
        }
        else if(command == COMMAND_MAX_RATE_ROLL){
          maxAngularRateRoll = value.toInt();
          Serial.print(command);
          Serial.print(" ");
          Serial.println(maxAngularRateRoll);
        } 
        else if(command == COMMAND_MAX_RATE_YAW){
          maxAngularRateYaw = value.toInt();
          Serial.print(command);
          Serial.print(" ");
          Serial.println(maxAngularRateYaw);
        }
        else if(command == COMMAND_PRINT_RATES){
          Serial.print("Rates(pitch,roll,yaw): ");
          Serial.print(maxAngularRatePitch);
          Serial.print(", ");
          Serial.print(maxAngularRateRoll);
          Serial.print(", ");
          Serial.println(maxAngularRateYaw);
        } 
        else if(command == COMMAND_EXPO_PITCH){
          expo[0] = atof(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(expo[0], 3);
        }
        else if(command == COMMAND_EXPO_ROLL){
          expo[1] = atof(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(expo[1], 3);
        }
        else if(command == COMMAND_EXPO_YAW){
          expo[2] = atof(value.c_str());
          Serial.print(command);
          Serial.print(" ");
          Serial.println(expo[2], 3);
        }
        else if(command == COMMAND_PRINT_EXPO){
          Serial.print("Expos(pitch; roll; yaw): ");
          Serial.print(expo[0],3);
          for(uint8_t i=1; i<3; i++){
            Serial.print("; ");
            Serial.print(expo[i],3);   
          }
          Serial.println();
        }
        else if(command == COMMAND_EXPO_FUNCTION){
          float temp = atof(value.c_str());
          Serial.print(command); 
          Serial.print(" ");
          Serial.println(temp,3);
          Serial.print("f(x)= (x-1500)*(1-");
          Serial.print(temp,3);
          Serial.print(")+500*((x-1500)/500)^3*");
          Serial.print(temp,3);
          Serial.println("+1500");
        }
#ifdef USE_EEPROM
        else if (command == COMMAND_SAVE_PID) {
          Serial.println(command);
          for (byte i = 0; i < 3; i++) {
            dataToBeSaved[i] = pidKp[flightMode][i];
            addressToBeSaved[i] = pidStartAddresses[flightMode] + (i * 2);
            dataToBeSaved[i + 3] = pidKi[flightMode][i];
            addressToBeSaved[i + 3] = pidStartAddresses[flightMode] + (i * 2) + 6;
            dataToBeSaved[i + 6] = pidKd[flightMode][i];
            addressToBeSaved[i + 6] = pidStartAddresses[flightMode] + (i * 2) + 12;
          }
          saveToEeprom = 9;
          if (!((dataInEeprom >> pidValueBits[flightMode])& 1)) {
            dataInEeprom = dataInEeprom | (1<<pidValueBits[flightMode]);
            EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
          }
        }
        else if (command == COMMAND_SAVE_ACC_CALIB) {
          Serial.println(command);
          if (accCalib == false) {
            for (byte i = 0; i < 3; i++) {
              dataToBeSaved[i] = (int)accAvg[i];
              addressToBeSaved[i] = ACC_CALIB_START_ADDRESS + i * 2;
            }
            saveToEeprom = 3;
            if (!((dataInEeprom >> ACC_CALIB)& 1)) {
              dataInEeprom = dataInEeprom | (1<<ACC_CALIB);
              EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
            }
          }
          else {
            Serial.println("Not calibrated");
          }
        }
        else if (command == COMMAND_SAVE_GYRO_CALIB) {
          Serial.println(command);
          if (gyroCalib == false) {
            for (byte i = 0; i < 3; i++) {
              dataToBeSaved[i] = (int)gyroOffset[i];
              addressToBeSaved[i] = GYRO_CALIB_START_ADDRESS + i * 2;
            }
            saveToEeprom = 3;
            if (!((dataInEeprom >> GYRO_CALIB)& 1)) {
              dataInEeprom = dataInEeprom | (1<<GYRO_CALIB);
              EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
            }
          }
          else {
            Serial.println("Not calibrated");
          }
        }
        else if(command == COMMAND_SAVE_MAX_RATES){
          Serial.println(command);
          dataToBeSaved[0] = maxAngularRatePitch;
          dataToBeSaved[1] = maxAngularRateRoll;
          dataToBeSaved[2] = maxAngularRateYaw;
          for(byte i=0; i<3; i++){
            addressToBeSaved[i] = MAX_RATES_START_ADDRESS + i * 2;
          }
          saveToEeprom = 3;
          if (!((dataInEeprom >> MAX_RATES)& 1)) {
              dataInEeprom = dataInEeprom | (1<<MAX_RATES);
              EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
          }
        }
        else if(command == COMMAND_SAVE_EXPO){
          Serial.println(command);
          for(uint8_t i=0; i<3; i++){
            dataToBeSaved[i] = (int16_t)(expo[i]*10000);
          }
          for(byte i=0; i<3; i++){
            addressToBeSaved[i] = EXPO_START_ADDRESS + i * 2;
          }
          saveToEeprom = 3;
          if (!((dataInEeprom >> EXPO)& 1)) {
              dataInEeprom = dataInEeprom | (1<<EXPO);
              EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
          }
        }
        else if (command == COMMAND_DELETE_ACC_CALIB) {
          Serial.println(command);
          dataInEeprom = dataInEeprom & ~(1<<ACC_CALIB);
          EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
        }
        else if (command == COMMAND_DELETE_GYRO_CALIB) {
          Serial.println(command);
          dataInEeprom = dataInEeprom & ~(1<<GYRO_CALIB);
          EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
        }
        else if (command == COMMAND_DELETE_PID) {
          Serial.println(command);
          dataInEeprom = dataInEeprom & ~(1<<pidValueBits[flightMode]);
          EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
        }
        else if(command == COMMAND_DELETE_MAX_RATES){
          Serial.println(command);
          dataInEeprom = dataInEeprom & ~(1<<MAX_RATES);
          EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
        }
        else if(command == COMMAND_DELETE_EXPO){
          Serial.println(command);
          dataInEeprom = dataInEeprom & ~(1<<EXPO);
          EEPROM[VALUES_IN_EEPROM_ADDRESS] = dataInEeprom;
        }
#endif
        else if (command == COMMAND_DO_ACC_CALIB) {
          Serial.println(command);
          accCalib = true;
        }
        else if (command == COMMAND_DO_GYRO_CALIB) {
          Serial.println(command);
          gyroCalib = true;
        }
        else if(command == COMMAND_BUZZER){
          Serial.println(command);
          buzzerFunc(100,100,20);
        }
        else {
          Serial.print("unknown command: ");
          Serial.println(command);
        }
      }
    }
  }
  for (byte i2 = 0; i2 < OUTPUT_AMOUNT; i2++) {
    order[i2] = i2;
  }

  for (byte counter = 0; OUTPUT_AMOUNT - (2 * counter) > 2; counter++) {
    smallest = counter;
    biggest = counter;
    for (byte i2 = (counter + 1); i2 < (OUTPUT_AMOUNT - counter); i2++) {
      if (outputVals[order[i2]] < outputVals[order[smallest]]) {
        smallest = i2;
      }
      if (outputVals[order[i2]] > outputVals[order[biggest]]) {
        biggest = i2;
      }
    }

    temp = order[smallest];
    order[smallest] = order[counter];
    order[counter] = temp;
    if (biggest == counter) {
      biggest = smallest;
    }

    temp = order[biggest];
    order[biggest] = order[OUTPUT_AMOUNT - counter - 1];
    order[OUTPUT_AMOUNT - counter - 1] = temp;

  }

#if ((OUTPUT_AMOUNT % 2) == 0) //If the amount of outputs is even, the last two numbers must be arranged. Doing this is slighlty faster than doing the above one more time

  if (outputVals[order[(OUTPUT_AMOUNT / 2) - 1]] > outputVals[order[(OUTPUT_AMOUNT / 2)]]) {
    temp = order[(OUTPUT_AMOUNT / 2) - 1];
    order[(OUTPUT_AMOUNT / 2) - 1] = order[(OUTPUT_AMOUNT / 2)];
    order[(OUTPUT_AMOUNT / 2)] = temp;
  }

#endif



  for (byte i = 0; i < (OUTPUT_AMOUNT - 1); i++) {
    if ((outputVals[order[i + 1]] - outputVals[order[i]]) < 20) {
      pinHigh(outputPins[order[i]]);
      outputTimes[order[i]] = microSeconds() + outputVals[order[i]];
      delayMicroseconds(15);
    }
    else {
      pinHigh(outputPins[order[i]]);
      outputTimes[order[i]] = microSeconds() + outputVals[order[i]];
    }
  }
  pinHigh(outputPins[order[OUTPUT_AMOUNT - 1]]);
  outputTimes[order[OUTPUT_AMOUNT - 1]] = microSeconds() + outputVals[order[OUTPUT_AMOUNT - 1]];

  //ALL OUTPUTS HIGH FOR A BIT LESS THAN 1 ms, TIME TO DO STUFF


  if (millis() > ((unsigned long)checkTimer + 100)) {
    checkTimer = millis();
    for (byte i = 0; i < CHANNEL_AMOUNT; i++) {
      if ((savedInputVals[i] > 2200) || (savedInputVals[i] < 800)) {
        signalIsGood = false;
        signalFound = false;
        break;
      }
    }
    if (signalFound == 1) {
      signalIsGood = true;
    }
  }

  if (updated == 0) {
    if ((updateTimer + 25) < millis()) {
      signalIsGood = false;
      signalFound = false;
      //   Serial.println("lost2");
      updateTimer = millis();
    }
  }
  else {
    updateTimer = millis();
    updated = 0;
  }


  if (armed) {
    if (abs((int)(savedInputVals[PPM_PITCH] - PITCH_MIDDLE)) < 8) {
      savedInputVals[PPM_PITCH] = PITCH_MIDDLE;
    }
    if (abs((int)(savedInputVals[PPM_ROLL] - ROLL_MIDDLE)) < 8) {
      savedInputVals[PPM_ROLL] = ROLL_MIDDLE;
    }
    if (abs((int)(savedInputVals[PPM_YAW] - YAW_MIDDLE)) < 8) {
      savedInputVals[PPM_YAW] = YAW_MIDDLE;
    }
    if (flightMode == 0) {
      error[0] = (int)(((int)(savedInputVals[PPM_PITCH] - PITCH_MIDDLE) * 8)/GYRO_FACTOR_PITCH) - (int)((float)gyroData[GYRO_X] * GYRO_SCALE);
      error[1] = (int)(((int)(savedInputVals[PPM_ROLL] - ROLL_MIDDLE) * 8)/GYRO_FACTOR_ROLL) - (int)((float)gyroData[GYRO_Y] * GYRO_SCALE);
    }
#ifdef AUTO_LEVEL_MODE
    else if (flightMode == 1) {
      error[0] = (int)(((int)(savedInputVals[PPM_PITCH] - PITCH_MIDDLE) * 8)/POS_FACTOR) - (int)((float)pos[0] * POS_SCALE);
      error[1] = (int)(((int)(savedInputVals[PPM_ROLL] - ROLL_MIDDLE) * 8)/POS_FACTOR) - (int)((float)pos[1] * POS_SCALE);
    }
#endif
    error[2] = (int)(((int)(savedInputVals[PPM_YAW] - YAW_MIDDLE) * 8)/GYRO_FACTOR_YAW) - (int)((float)gyroData[GYRO_Z] * GYRO_SCALE);



    for (byte i = 0; i < 3; i++) {
      PID_P[i] = (int)(((long)error[i] * (long)pidKp[flightMode][i]) / (((long)4000 * 500) / MAX_CORRECTION)); //The error is 4000 when there is no gyro movement and the roll/pitch/yaw values are at max. It's multiplied by 500 to acccount for the PID_Kp value (if it's 500 they cancel out)
      PID_I[i] += (int)(((long)error[i] * (long)pidKi[flightMode][i]) / (((long)400000 * 500) / MAX_CORRECTION));
      PID_I[i] = constrain(PID_I[i], -150 , 150);
      PID_D[i] = (int)((((long)(((long)(error[i] - lastError[i]) * (long)pidKd[flightMode][i]) / (((long)4000 * 500) / MAX_CORRECTION))*128)+(long)PID_D[i]*(256-128))/256);
      lastError[i] = error[i];
    }


    for (byte i = 0; i < 3; i++) {
      correction[i] = (int)(PID_P[i] + PID_I[i] + PID_D[i]);
    }

    motorCorrection[0] = (-1 * correction[0]);
    motorCorrection[1] = (-1 * correction[0]);
    motorCorrection[2] = correction[0];
    motorCorrection[3] = correction[0];

    motorCorrection[0] += correction[1];
    motorCorrection[1] += (-1 * correction[1]);
    motorCorrection[2] += (-1 * correction[1]);
    motorCorrection[3] += correction[1];

    motorCorrection[0] += (-1 * correction[2]);
    motorCorrection[1] += correction[2];
    motorCorrection[2] += (-1 * correction[2]);

    int overMax = 0;
    int overMin = 0;
    for (byte i = 0; i < 4; i++) {
      int temp = ((int)(savedInputVals[PPM_THROTTLE] + motorCorrection[i]) - THROTTLE_MAX);
      if (temp > overMax) {
        overMax = temp;
      }
    }
    for (byte i = 0; i < 4; i++) {
      int temp = ( THROTTLE_MIN - (int)(savedInputVals[PPM_THROTTLE] + motorCorrection[i]));
      if (temp > overMin) {
        overMin = temp;
      }
    }
    throttleOffset = ((0-overMax+overMin));//*4+throttleOffset*(256-4))/256;
    throttleOffset = constrain(throttleOffset, -400,400);
    int tempThrottle = savedInputVals[PPM_THROTTLE]+throttleOffset;
   /* if(throttleOffset > 0){
      tempThrottle += 25;
    }*/
    
    for (byte i = 0; i < 4; i++) {
      outputVals[i] = constrain(tempThrottle + motorCorrection[i], THROTTLE_MIN, THROTTLE_MAX);
    }
  }

  //END OF STUFF

  for (byte i = 0; i < (OUTPUT_AMOUNT); i++) {
    while (outputTimes[order[i]] > microSeconds()) {}
    pinLow(outputPins[order[i]]);
  }


#ifdef AUTO_LEVEL_MODE
  for (byte i = 0; i < 2; i++) {
    pos[i] += (((long)gyroData[gyroXYZ[i]] * loopTime)) / GYRO_SCALE_FOR_AL;
  }

  sinGyroZ = (int)(32768 * sin(((float)gyroData[GYRO_Z] * loopTime) * 0.000000000532632)); //  1/(32768000*(180/PI)) = 0.000000000532632
  temp2 = pos[1];
  pos[1] -= ((pos[0] * (long)sinGyroZ) / 32768);
  pos[0] += ((temp2 * (long)sinGyroZ) / 32768);
 // hyp = (unsigned long)sqrt((uint64_t)((long)accData[ACC_X] * accData[ACC_X]) + ((long)accData[ACC_Y] * accData[ACC_Y]) + ((long)accData[ACC_Z] * accData[ACC_Z]));

/*
  if (abs((long)(normalHyp - hyp)) > 1000) {
    accFact = 4;
    //Serial.println(normalHyp);
    //Serial.println(hyp);
  }
  else {
    accFact = 32;
  }*/
  //accPosX = ((long)(1000.0 * 57.29578 * atan((float)accData[ACC_X] / (float)accData[ACC_Z])) * (accFact) + accPosX * (256 - accFact)) / 256; //256-64 = 192
  //accPosY = ((long)(1000.0 * 57.29578 * atan((float)accData[ACC_Y] / (float)accData[ACC_Z])) * (accFact) + accPosY * (256 - accFact)) / 256;
  pos[1] = (long)((long)(pos[1] * (8192 - accFact)) + ((long)accFact * ((long)(1000.0 * 57.29578 * atan((float)accData[ACC_X] / (float)accData[ACC_Z])) - posOffset[1]))) / 8192; //   180/PI= 57.29578
  pos[0] = (long)((long)(pos[0] * (8192 - accFact)) + ((long)accFact * ((long)(1000.0 * 57.29578 * atan((float)accData[ACC_Y] / (float)accData[ACC_Z])) - posOffset[0]))) / 8192;
  if ((printTimer + 100) < millis()) {
    printTimer = millis();
    for(byte i= 0; i<2; i++){
       Serial.print(pos[i]);
       Serial.print("\t");
      }
      Serial.println();
  }
      /*
    Serial.print(accPosX);
    Serial.print("\t");
    Serial.print(accPosY);
    Serial.print("\t");
    
      Serial.print(accData[ACC_X]);
      Serial.print("\t");
      Serial.print(accData[ACC_Y]);
      Serial.print("\t");
      Serial.print(accData[ACC_Z]);
      Serial.print("\t");
      Serial.print(hyp);
      Serial.print("\t");
    
    Serial.println();
  }*/
#endif
/*
if ((printTimer + 100) < millis()) {
    printTimer = millis();
    for(byte i=0; i<3; i++){
      Serial.print(gyroData[i]);
      Serial.print("\t");
    }
    Serial.println();
}
*/

  
  if (inputValsUpdated) {
    inputValsUpdated = false;
    inputValsUpdatedTimer = millis();
    noInterrupts();
    for (byte i = 0; i < CHANNEL_AMOUNT + 1; i++) {
      savedInputVals[i] = inputVals[i];
    }
    interrupts();
  
 //   Serial.print(savedInputVals[PPM_PITCH]);
  //  Serial.print("\t");
    savedInputVals[PPM_PITCH] = calculateExpo((int16_t)savedInputVals[PPM_PITCH]-PITCH_MIDDLE, PITCH_MAX-PITCH_MIDDLE, PITCH_MIDDLE, expo[0]);
  /*  Serial.print(savedInputVals[PPM_PITCH]);
    Serial.print("\t");
    Serial.print(savedInputVals[PPM_ROLL]);
    Serial.print("\t");*/
    savedInputVals[PPM_ROLL] = calculateExpo((int16_t)savedInputVals[PPM_ROLL]-ROLL_MIDDLE, ROLL_MAX-ROLL_MIDDLE, ROLL_MIDDLE, expo[1]);
  /*  Serial.print(savedInputVals[PPM_ROLL]);
    Serial.print("\t");
    Serial.print(savedInputVals[PPM_YAW]);
    Serial.print("\t");*/
    savedInputVals[PPM_YAW] = calculateExpo((int16_t)savedInputVals[PPM_YAW]-YAW_MIDDLE, YAW_MAX-YAW_MIDDLE, YAW_MIDDLE, expo[2]);
  /*  Serial.print(savedInputVals[PPM_YAW]);
    Serial.print("\t");
    Serial.println();*/
   /*
    if((savedInputVals[PPM_THROTTLE] < (THROTTLE_MIN + 150)) && armed){
      savedInputVals[PPM_THROTTLE] = THROTTLE_MIN + 150;
    }*/
  }
  else {
    if ((inputValsUpdatedTimer + 150) < millis()) {
      signalIsGood = false;
      //   Serial.println("lost3");
      inputValsUpdatedTimer = millis();
    }
  }
  /* if (!signalIsGood) {
     Serial.println("inputVals: ");
     for (byte i = 0; i < CHANNEL_AMOUNT + 1; i++) {
       Serial.println(inputVals[i]);
     }
     Serial.println();
     Serial.println("savedInputVals: ");
     for (byte i = 0; i < CHANNEL_AMOUNT + 1; i++) {
       Serial.println(savedInputVals[i]);
     }
     Serial.println();
    }*/


#ifdef BUZZER
  if (buzzer[2] > 0) {
    if (millis() > buzzerTimer) {
      if (buzzerOn) {
        pinLow(BUZZER_PIN);
        buzzerOn = false;
        buzzerTimer = millis() + buzzer[1];
        buzzer[2]--;
      }
      else {
        pinHigh(BUZZER_PIN);
        buzzerOn = true;
        buzzerTimer = millis() + buzzer[0];
      }
    }
  }
#endif
  while ((startTime + loopWaitTime) > microSeconds()) {}
}




ISR(TIMER1_OVF_vect) {
  t1Ovf++;
}

unsigned long microSeconds() {
  return ((unsigned long)(TCNT1 / 2) + (32768 * t1Ovf));
}


void pinLow(byte pin) {
  if (pin < 8) {
    PORTD &= ~(1 << pin);
  }
  else if (pin > 19) {
    PORTC &= ~(1 << (pin - 20));
  }
  else {
    PORTB &= ~(1 << (pin - 8));
  }
}

void pinHigh(byte pin) {
  if (pin < 8) {
    PORTD |= 1 << pin;
  }
  else if (pin > 19) {
    PORTC |= 1 << (pin - 20);
  }
  else {
    PORTB |= 1 << (pin - 8);
  }
}

void setAsOutput(byte pin) {
  if (pin < 8) {
    DDRD |= 1 << pin;
  }
  else if (pin > 19) {
    DDRC |= 1 << (pin - 20);
  }
  else {
    DDRB |= 1 << (pin - 8);
  }
}


ISR(PPM_INTERRUPT) {
  updated = 1;
  unsigned int tempTCNT1 = TCNT1;
  if (TIFR1 & 1) {
    signalTimer1 = microSeconds() + 32768;
  }
  else {
    signalTimer1 = ((unsigned long)(tempTCNT1 / 2) + (32768 * t1Ovf));  //if TCNT1 overflows between the if statement and this line of code, the values would be incorrect. This way it works.
  }

  if (signalFound == 0) {
    if ((signalTimer1 - signalTimer2) > 7000) {
      dataLevel = !(PPM_PIN_STATE);
      channel = 0;
      signalFound = 1;
    }
    signalTimer2 = signalTimer1;
  }
  else {
    if (channel <= CHANNEL_AMOUNT) {
      if (PPM_PIN_STATE != dataLevel) {
        inputVals[channel] = (signalTimer1 - signalTimer2);
        signalTimer2 = signalTimer1;
        channel++;
      }
    }
    else {
      channel = 0;
      inputValsUpdated = true;
    }
  }
}


#ifdef USE_EEPROM
int eepromReadInt(int memoryAddress) {
  return ((int)EEPROM[memoryAddress] << 8) | (int)EEPROM[memoryAddress + 1];
}

void eepromWriteInt(int memoryAddress, int dataToBeSaved) {
  EEPROM[memoryAddress] = (byte)((dataToBeSaved >> 8) & (int)0b0000000011111111);
  EEPROM[memoryAddress + 1] = (byte)(dataToBeSaved & (int)0b0000000011111111);
}
#endif

#ifdef BUZZER
void buzzerFunc(unsigned int onTime, unsigned int offTime, unsigned int repeats) {
  buzzer[0] = onTime;
  buzzer[1] = offTime;
  buzzer[2] = repeats;
}
#endif

uint16_t calculateExpo(int16_t value, int16_t maxVal,int16_t middleVal, float expoVal){
    return (uint16_t)((int16_t)((float)value*(1.0-expoVal)+ value*((float)value/(float)maxVal)*((float)value/(float)maxVal)*expoVal) + middleVal);
}
void auxSwitches() {
  if (ARM_CHECK && (gyroCalib == false) && (accCalib == false) && (savedInputVals[PPM_THROTTLE] < 1300) && signalIsGood) {
    if((armed == false) && armTimer == 4294966295){   // 4294966295 = (2^32)-1-1000
      #ifdef BUZZER
      buzzerFunc(125, 125, 4);
      #endif
      armTimer = millis();
    }
    else if((armTimer + 1000) < millis()){
      armed = true;
      loopWaitTime = 4000;
      armTimer = 4294966295;
    }
  }
  else if(!(ARM_CHECK)){
    armTimer = 4294966295;
    if(armed == true){
      armed = false;
      for (byte i = 0; i < 4; i++) {
        outputVals[i] = 900;
      }
      throttleOffset = 0;
      loopWaitTime = 20000;
      for (byte i = 0; i < 3; i++) {
        PID_P[i] = 0;
        PID_I[i] = 0;
        PID_D[i] = 0;
      }
      while(Serial.available()){
        Serial.read();
      }
    }
  }
#ifdef AUTO_LEVEL_MODE
  if (AUTO_LEVEL_ON_CHECK && signalIsGood) {
    if (flightMode != 1) {
      flightMode = 1;
#ifdef BUZZER
      buzzerFunc(100, 0, 1);
#endif
    }
  }
  else if (!(AUTO_LEVEL_ON_CHECK)) {
    if (flightMode != 0) {
      flightMode = 0;
#ifdef BUZZER
      buzzerFunc(100, 0, 1);
#endif
    }
  }
#endif
#ifdef BUZZER_ALARM_CHANNEL
  if(BUZZER_ALARM_CHECK && signalIsGood){
    buzzerFunc(100,100,9999);
    buzzerAlarm = true;
  }
  else if(buzzerAlarm == true){
      buzzerFunc(0,0,1);
      buzzerAlarm = false;
  }
#endif
}

