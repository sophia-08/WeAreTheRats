#pragma once

int imuInit(int deviceMode);
int imuReadAndUpdateXYAngle();
void imuConfigure(int deviceMode);
int imuReadNoWait();
int imuSaveData(int samplesRead);

#define accl_min -30.0
#define accl_max 30.0
#define gyro_min -15.0
#define gyro_max 15.0
#define roto_min -3.0
#define roto_max 2.0

typedef struct euler_t {
  float yaw;
  float pitch;
  float roll;
} euler;
void displayData();
float imuSumOfAbsolateAcclOfAllAxis();
bool imuDataReady();
void imuStartSave(bool start);
bool imuPreprocessData();
void imuDisplayPixelArray();