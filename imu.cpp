
#define BNO085
// #define IMU_USE_RESET
#define BNO08X_RESET -1

#define IMU_USE_INT
// #include "LSM6DS3.h"

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

#include "local_constants.h"
#include "system.h"
extern int deviceMode;
extern float xAngle, yAngle, lastXAngle, lastYAngle;
extern int samplesRead;

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

void setReports() {
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

int initIMU() {

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    systemHaltWithledPattern(LED_RED, 3);
  }
  Serial.println("BNO08x Found!");

  setReports();
  delay(100);

  return 0;
}

int readIMUNoWait() { return bno08x.getSensorEvent(&sensorValue); }

int readIMUAndUpdateXYAngle() {

  // BNO085 pull IMU_INT LOW when data is ready
  // so do nothing in case of IMU_INT high
#ifdef IMU_USE_INT
  if (digitalRead(IMU_INT) == HIGH) {
    return 1;
    // systemSleep();
  }
#endif
  static uint32_t last = 0;
  long now = micros();

  readIMUNoWait();
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

int saveData() {
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

float sumAbsolateAcclOfAllAxis() {
    return abs(accl[0]) + abs(accl[1]) + abs(accl[2]);
}
#endif
