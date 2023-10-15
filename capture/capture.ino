// #include "LSM6DS3.h"
#include "Adafruit_BNO055.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Create a instance of class LSM6DS3
// LSM6DS3 myIMU(I2C_MODE, 0x6A);         // I2C device address 0x6A
float accelX, accelY, accelZ, // units m/s/s i.e. accelZ if often 9.8 (gravity)
    gyroX, gyroY, gyroZ,      // units dps (degrees per second)
    gyroDriftX, gyroDriftY, gyroDriftZ; // units dps
float gyroRoll, gyroPitch, gyroYaw,     // units degrees (expect major drift)
    gyroCorrectedRoll, gyroCorrectedPitch,
    gyroCorrectedYaw, // units degrees (expect minor drift)
    accRoll, accPitch,
    accYaw, // units degrees (roll and pitch noisy, yaw not possible)
    complementaryRoll, complementaryPitch,
    complementaryYaw; // units degrees (excellent roll, pitch, yaw minor drift)

#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D6
#define DEBUG_2 D2
#define DEBUG_3 D3
#define IMU_RESET D0

const float accelerationThreshold = 1.6; // threshold of significant in G's
const int numSamples = 500;
double samples[numSamples][6];
int samplesRead = 0;
int startTime, currentTime;
uint8_t readData;

// Check I2C device address and correct line below (by default address is 0x29
// or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // Reset IMU
  digitalWrite(IMU_RESET, HIGH);
  digitalWrite(IMU_RESET, LOW);
  digitalWrite(IMU_RESET, HIGH);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(DEBUG_2, OUTPUT);
  pinMode(DEBUG_3, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  // pinMode(LED_CHARGER, OUTPUT);
  pinMode(D6, INPUT_PULLUP);
  pinMode(D7, INPUT_PULLUP);
  pinMode(D8, INPUT_PULLUP);
  pinMode(D9, INPUT_PULLUP);
  pinMode(D10, INPUT_PULLUP);

  digitalWrite(D6, HIGH);
  digitalWrite(D7, HIGH);
  digitalWrite(D8, HIGH);
  digitalWrite(D9, HIGH);
  digitalWrite(D10, HIGH);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);
  // digitalWrite(LED_CHARGER, LOW);

  // Call .begin() to configure the IMUs
  // if (myIMU.begin() != 0) {
  //   Serial.println("Device error");
  //   while (1)
  //     ;
  // }
  // Wire1.setClock(400000UL);  // SCL 400kHz  //
  // Wire.setClock(400000UL);  // SCL 400kHz
  startTime = micros();
  calibrateIMU(0, 0);

  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1)
      ;
  }
}

void calibrateIMU(int delayMillis, int calibrationMillis) {
  gyroDriftX = 0; // 0.67;
  gyroDriftY = 0; //-1.38;
  gyroDriftZ = 0; //-0.51;
  return;
}

sensors_event_t orientationData, linearAccelData, angVelData;
bool readIMU() {

  digitalWrite(DEBUG_3, HIGH);
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  digitalWrite(DEBUG_3, LOW);
  return true;
}

int count = 0;
float sumX, sumY, sumZ;

#define PRECISION 4
float lastAx, lastAy, lastAz;
bool startedChar = false;
int t1 = 0;
bool d2;

void loop() {

  // Capture has not started, ignore until user activate keypad
  if (!startedChar) {
    if (digitalRead(MOUSE_ACTIVATE) == LOW) {
      return;
    } else {
      // User activate keypad, check whether 2s passed since last capture
      int currentTime = millis();
      if (currentTime < t1 + 2000) {
        return;
      }
      startedChar = true;
      t1 = currentTime;
      samplesRead = 0;
    }
  }

  digitalWrite(LED_BLUE, HIGH);

  while (true) {
  wait:
    // User deactivated keypad
    if (digitalRead(MOUSE_ACTIVATE) == LOW) {
      startedChar = false;
      break;
    }
    readIMU();
    if (linearAccelData.acceleration.x == lastAx &&
        linearAccelData.acceleration.y == lastAy &&
        linearAccelData.acceleration.z == lastAz) {
      delay(0.5);
      goto wait;
    } else {
      lastAx = linearAccelData.acceleration.x;
      lastAy = linearAccelData.acceleration.y;
      lastAz = linearAccelData.acceleration.z;
      samples[samplesRead][0] = linearAccelData.acceleration.x;
      samples[samplesRead][1] = linearAccelData.acceleration.y;
      samples[samplesRead][2] = linearAccelData.acceleration.z;
      samples[samplesRead][3] = angVelData.gyro.x;
      samples[samplesRead][4] = angVelData.gyro.y;
      samples[samplesRead][5] = angVelData.gyro.z;
      Serial.print(samplesRead);
      Serial.print(',');
      for (int i = 0; i < 5; i++) {
        Serial.print(samples[samplesRead][i], PRECISION);
        Serial.print(',');
      }
      Serial.println(samples[samplesRead][5], PRECISION);
      samplesRead++;
      d2 = !d2;
      digitalWrite(DEBUG_2, d2);
    }
    if (samplesRead >= numSamples) {
      samplesRead = 0;
    }
  }

  // for  (int ss=0; ss< samplesRead ; ss++) {

  //   // read the acceleration and gyroscope data
  //   // print the data in CSV format
  //   Serial.print(ss);
  //   Serial.print(',');
  //   for (int i = 0; i < 5; i++) {
  //     Serial.print(samples[ss][i], PRECISION);
  //     Serial.print(',');
  //   }
  //   Serial.println(samples[ss][5], PRECISION);
  // }
  // Serial.println();

  digitalWrite(LED_BLUE, LOW);
}

void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch =
      atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

  float lastFrequency = 416; //(float)1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll =
      gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch =
      gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll =
      complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch =
      complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
}