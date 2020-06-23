#ifndef SENSORSH
#define SENSORSH
byte sensorSetup();
void getAcc();
void getGyro();
extern int gyroData[];
extern int accData[];
#endif //SENSORSH
