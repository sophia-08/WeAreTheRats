#pragma once

int initIMU();
int readIMUAndUpdateXYAngle();
void setReports();
int readIMUNoWait();
int saveData();

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
void displayData() ;
float sumAbsolateAcclOfAllAxis();