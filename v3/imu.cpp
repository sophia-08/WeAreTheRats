

// #include "LSM6DS3.h"
#include "local_constants.h"

#include "system.h"
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
#ifdef PIMORONI_TRACKBALL
  xAngle = Orientation[0];
  yAngle = -Orientation[2];
#else
  xAngle = -Orientation[2];
  yAngle = Orientation[0];
#endif
  if (bWrite) {
    writeTrajectory[writeIndex][0] = xAngle;
    writeTrajectory[writeIndex][1] = yAngle;
    writeIndex++;
    if (writeIndex > 1999) {
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
  } else {
    bWrite = false;
  }
}

bool imuPreprocessData() {
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
      } else {
        Serial.print(".");
      }
    }
    Serial.println("");
  }
}
void imuDisplayPixelArray() { dump((unsigned char *)buf, 32 * 32); }
#endif
