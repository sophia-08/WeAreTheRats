

// #include "LSM6DS3.h"
#include "local_constants.h"

#include "system.h"

#ifdef BNO085
#include "Adafruit_BNO08x.h"
#include <Wire.h>

#include "imu.h"
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
extern TfLiteTensor *tflInputTensor;
extern int tensorIndex;

extern float xAngle, yAngle;

// New data available. external set to true in adafruit_bno08x.cpp
// sensorHandler() Currently for keyboard, new data available every 10ms; for
// mouse, every 20ms
bool newData = false;

// Rotation Vector. i, j, k, real
// external updated in adafruit_bno08x.cpp sensorHandler()
float rtVector[4];

// Linear acceleration, x, y, z
// external updated in adafruit_bno08x.cpp sensorHandler()
float accl[3];

// gyro, x, y, z
// external updated in adafruit_bno08x.cpp sensorHandler()
float gyro[3];

// calibration status
// external updated in adafruit_bno08x.cpp sensorHandler()
int calStatus;

euler ypr, ypr0;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
// sh2_SensorId_t reportType = SH2_ROTATION_VECTOR; // SH2_ARVR_STABILIZED_RV;
// long reportIntervalUs = 20000;

void imuConfigure(int deviceMode) {
  int dataRate;
  if (deviceMode == DEVICE_MOUSE_MODE) {
    dataRate = 20 * 1000;
  } else {
    dataRate = 10 * 1000;
  }

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, dataRate)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, dataRate)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, dataRate)) {
    Serial.println("Could not enable gyroscope");
  }
}

void quaternionToEuler(float qi, float qj, float qk, float qr, euler *ypr,
                       bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

int imuInit(int deviceMode) {

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    systemHaltWithledPattern(LED_RED, 3);
  }
  Serial.println("BNO08x Found!");

  imuConfigure(deviceMode);
  delay(100);

  return 0;
}

int imuReadNoWait() { return bno08x.getSensorEvent(&sensorValue); }

int imuReadAndUpdateXYAngle() {

  // BNO085 pull IMU_INT LOW when data is ready
  // so do nothing in case of IMU_INT high
  // #ifdef IMU_USE_INT
  //   if (digitalRead(IMU_INT) == HIGH) {
  //     return 1;
  //     // systemSleep();
  //   }
  // #endif
  static uint32_t last = 0;
  long now = micros();

  imuReadNoWait();
  if (newData) {
    newData = false;
    displayData();
    quaternionToEuler(rtVector[0], rtVector[1], rtVector[2], rtVector[3], &ypr,
                      true);
    xAngle = -ypr.yaw;
    yAngle = ypr.roll;
  }

  return 0;
}

#define pi 3.14159265358979323846
float calRotation(float x, float x0) {
  float tmp1;
  tmp1 = x - x0;
  if (tmp1 > pi) {
    tmp1 -= pi * 2;
  }
  if (tmp1 < -pi) {
    tmp1 += pi * 2;
  }
  return tmp1;
}

int imuSaveData(int samplesRead) {
  quaternionToEuler(rtVector[0], rtVector[1], rtVector[2], rtVector[3], &ypr,
                    false);

  if (samplesRead == 0) {
    ypr0 = ypr;
  }
  tflInputTensor->data.f[tensorIndex++] =
      (accl[0] - accl_min) / (accl_max - accl_min);
  tflInputTensor->data.f[tensorIndex++] =
      (accl[1] - accl_min) / (accl_max - accl_min);
  tflInputTensor->data.f[tensorIndex++] =
      (accl[2] - accl_min) / (accl_max - accl_min);
  tflInputTensor->data.f[tensorIndex++] =
      (gyro[0] - gyro_min) / (gyro_max - gyro_min);
  tflInputTensor->data.f[tensorIndex++] =
      (gyro[1] - gyro_min) / (gyro_max - gyro_min);
  tflInputTensor->data.f[tensorIndex++] =
      (gyro[2] - gyro_min) / (gyro_max - gyro_min);

  tflInputTensor->data.f[tensorIndex++] =
      (calRotation(ypr.yaw, ypr0.yaw) - roto_min) / (roto_max - roto_min);
  tflInputTensor->data.f[tensorIndex++] =
      (calRotation(ypr.pitch, ypr0.pitch) - roto_min) / (roto_max - roto_min);
  tflInputTensor->data.f[tensorIndex++] =
      (calRotation(ypr.roll, ypr0.roll) - roto_min) / (roto_max - roto_min);
  return 0;
}

void displayData() {
  // uint32_t now = micros();

  // Serial.print(now - last);
  // Serial.print("\t");
  // last = now;
  // Serial.print(calStatus);
  // Serial.print("\t");
  // // This is accuracy in the range of 0 to 3
  // int i;
  // for (i = 0; i < 4; i++) {
  //   Serial.print("\t");
  //   Serial.print(rtVector[i]);
  // }
  // for (i = 0; i < 3; i++) {
  //   Serial.print("\t");
  //   Serial.print(accl[i]);
  // }
  // for (i = 0; i < 3; i++) {
  //   Serial.print("\t");
  //   Serial.print(gyro[i]);
  // }
  // Serial.println("");
}

float imuSumOfAbsolateAcclOfAllAxis() {
  return abs(accl[0]) + abs(accl[1]) + abs(accl[2]);
}

bool imuDataReady() {
  // BNO085 pull IMU_INT LOW when data is ready
  if (digitalRead(IMU_INT) == LOW)
    return true;
  return false;
}
#endif

#ifdef IMU_LSM6DS3

#include "LSM6DS3.h"
#include <Wire.h>
#include <bluefruit.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);
extern float xAngle, yAngle;
void calibrateIMU(int delayMillis, int tries);
void doCalculations();
bool readIMU();
void printOrientation();
float gDrift[3], Orientation[3];
float g[3];

bool bWrite;
float writeTrajectory[2000][2];
int writeIndex;
char buf[32][32];
int bufSize;

int imuInit(int deviceMode) {
  // set 6ds int1 as input
  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);

  if (myIMU.begin() != 0) {
    Serial.println("IMU error");
    systemHaltWithledPattern(LED_RED, 3);
  }

  calibrateIMU(100, 1000);
  return 0;
}
int imuReadAndUpdateXYAngle() {
  readIMU();
  doCalculations();
  // printOrientation();

  xAngle = -Orientation[2];
  yAngle = Orientation[0];

    if (bWrite) {
      writeTrajectory[writeIndex][0] = xAngle;
      writeTrajectory[writeIndex][1] = yAngle;
      writeIndex++;
      if (writeIndex>1999) {
        writeIndex = 1999;
      }
    }

  return 0;
}
void imuConfigure(int deviceMode) { return; }
int imuReadNoWait() { return 0; };
int imuSaveData(int samplesRead) { return 0; }

void displayData() { return; }
float imuSumOfAbsolateAcclOfAllAxis() { return 0; }
bool imuDataReady() {
  // PIN_LSM6DS3TR_C_INT1 go high when IMU data is ready, and stay high until
  // data is readen
  if (digitalRead(PIN_LSM6DS3TR_C_INT1)) {
    return true;
  } else
    return false;
}

/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when
  arduino is still we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int tries) {
  // 0.88580,-1.40338,-1.19635
  gDrift[0] = 0.88892;
  gDrift[1] = -1.41071;
  gDrift[2] = -1.19446;
  return;

  int calibrationCount = 0;
  int i;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sum[3] = {0, 0, 0};
  // int startTime = millis();
  while (calibrationCount < tries) {

    if (digitalRead(PIN_LSM6DS3TR_C_INT1)) {
      readIMU();
      sum[0] += g[0];
      sum[1] += g[1];
      sum[2] += g[2];
      calibrationCount++;
    }
  }

  gDrift[0] = sum[0] / tries;
  gDrift[1] = sum[1] / tries;
  gDrift[2] = sum[2] / tries;

  for (i = 0; i < 3; i++) {

    Serial.print(gDrift[i], 5);
    Serial.print(",");
  }
  Serial.println("");
}

bool readIMU() {

  // At 400k clock, read register one by one
  //  read 3 registers, took total 467us
  //  read 6 registers, took total 927us
  //  imu data ready every 2.32ms
  // use readRegisterRegion
  // took 403us at i2c 400khz clock, 1.49ms at 100khz clock

  uint8_t data[12];
  myIMU.readRegisterRegion(data, 0x22, 6);
  int16_t *p = (int16_t *)data;
  g[0] = *p * 2000.0 / 32768.0;
  p++;
  g[1] = *p * 2000.0 / 32768.0;
  p++;
  g[2] = *p * 2000.0 / 32768.0;
  //   p++;
  //   a[0] = *p * 4.0 / 32768.0;
  //   p++;
  //   a[1] = *p * 4.0 / 32768.0;
  //   p++;
  //   a[2] = *p * 4.0 / 32768.0;
  //   p++;

  return true;
}

void doCalculations() {

  float freq = 416.0;
  for (int i = 0; i < 3; i++) {
    Orientation[i] += (g[i] - gDrift[i]) / freq;
  }
}

void printOrientation() {
  for (int i = 0; i < 3; i++) {

    Serial.print(Orientation[i], 1);
    Serial.print(",");
  }
  Serial.println("");
}

void imuStartSave(bool start) {
  if (start) {
    bWrite = true;
    writeIndex = 0;
  }else{
    bWrite = false;
  }
  
}

bool imuPreprocessData(){
Serial.print("index ");
Serial.println(writeIndex);

  if (writeIndex < 200) {
    return false;
  }

  float xmin = 1e9, ymin = 1e9;
  float xmax = -1e9, ymax = -1e9;
#define TRIM_LEFT 30
#define TRIM_RIGH 30
  for (int i = TRIM_LEFT; i < writeIndex - TRIM_RIGH; i++) {
    if (writeTrajectory[i][0] < xmin) {
      xmin = writeTrajectory[i][0];
    };
    if (writeTrajectory[i][0] > xmax) {
      xmax = writeTrajectory[i][0];
    };
    if (writeTrajectory[i][1] < ymin) {
      ymin = writeTrajectory[i][1];
    };
    if (writeTrajectory[i][1] > ymax) {
      ymax = writeTrajectory[i][1];
    };
  }
  bufSize = 32 * 32;
  memset(buf, 0, bufSize);

  float xrange = xmax - xmin;
  float yrange = ymax - ymin;
  float mrange = 20;

  if (xrange < mrange)
    xrange = mrange;
  if (yrange < mrange)
    yrange = mrange;

  for (int i = TRIM_LEFT; i < writeIndex - TRIM_RIGH; i++) {
    int x = int((writeTrajectory[i][0] - xmin) / xrange * 31) % 32;
    int y = int((writeTrajectory[i][1] - ymin) / yrange * 31) % 32;
    buf[y][x] = 1;
  }
  return true;
}

// void dump(unsigned char *buf, int size) {
//   unsigned char *p = buf;
//   int rows = size / 32;
//   int remains = size % 32;
//   int i, j;
//   for (i = 0; i < rows; i++) {
//     Serial.print(i);
//     for (j = 0; j < 32; j++) {
//       Serial.print(",");
//       Serial.print(*p++, HEX);
//     }
//     Serial.println("");
//   }
//   if (remains) {
//     Serial.print(i);
//     for (j = 0; j < remains; j++) {
//       Serial.print(",");
//       Serial.print(*p++, HEX);
//     }
//     Serial.println("");
//   }
// }
void dump(unsigned char *buf, int size) {
  unsigned char *p = buf;
  int rows = size / 32;
  int remains = size % 32;
  int i, j;
  for (i = 0; i < rows; i++) {
    for (j = 0; j < 32; j++) {
      if (*p++ == 0) {
        Serial.print(" ");
      }else{
        Serial.print(".");
      }
    }
    Serial.println("");
  }
}
void imuDisplayPixelArray() {
  dump((unsigned char *)buf, 32*32);
}
#endif
