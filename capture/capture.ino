#include "LSM6DS3.h"
#include <Wire.h>

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);         // I2C device address 0x6A
float accelX, accelY, accelZ,          // units m/s/s i.e. accelZ if often 9.8 (gravity)
  gyroX, gyroY, gyroZ,                 // units dps (degrees per second)
  gyroDriftX, gyroDriftY, gyroDriftZ;  // units dps
#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D6
const float accelerationThreshold = 1.8;  // threshold of significant in G's
const int numSamples = 416;
int samplesRead = 0;
int startTime, currentTime;
uint8_t readData;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // while (!Serial)
  //   ;
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
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
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
    while (1)
      ;
  }
  Wire1.setClock(400000UL);  // SCL 400kHz
  startTime = micros();
  calibrateIMU(0,0);
}

void calibrateIMU(int delayMillis, int calibrationMillis) {
  gyroDriftX = 0.67;
  gyroDriftY = -1.38;
  gyroDriftZ = -0.51;
  return;
}

bool readIMU() {
  uint8_t readData;
  uint8_t data[12];
  myIMU.readRegisterRegion(data, 0x22, 12);
  int16_t *p = (int16_t *)data;
  gyroX = *p * 2000.0 / 32768.0;
  p++;
  gyroY = *p * 2000.0 / 32768.0;
  p++;
  gyroZ = *p * 2000.0 / 32768.0;
  p++;
  accelX = *p * 4.0 / 32768.0;
  p++;
  accelY = *p * 4.0 / 32768.0;
  p++;
  accelZ = *p * 4.0 / 32768.0;

  return true;
}

int count = 0;
float sumX, sumY, sumZ;
void loop() {
  if (digitalRead(MOUSE_ACTIVATE) == LOW) {
    return;
  }
  digitalWrite(LED_BLUE, HIGH);
  samplesRead = 0;
while (samplesRead < numSamples) {
wait:
  myIMU.readRegister(&readData, LSM6DS3_ACC_GYRO_STATUS_REG);  //0,0,0,0,0,TDA,GDA,XLDA
  if ((readData & 0x07) != 0x07) goto wait;
  readIMU();
  // count++;
  // sumX += gyroX;
  // sumY += gyroY;
  // sumZ += gyroZ;
  // if (count % 100 == 0) {
  //   currentTime = micros();
  //   Serial.println(currentTime - startTime);
  //   startTime = currentTime;
  //   Serial.print(sumX / 100);
  //   Serial.print(",");
  //   Serial.print(sumY / 100);
  //   Serial.print(",");
  //   Serial.println(sumZ / 100);
  //   sumX = 0;
  //   sumY = 0;
  //   sumZ = 0;
  // }
  // return;
  // wait for significant motion
  // if (samplesRead == numSamples) {
  //   // sum up the absolutes
  //   float aSum = fabs(accelX) + fabs(accelY) + fabs(accelZ);

  //   // check if it's above the threshold
  //   if (aSum < accelerationThreshold) {
  //     return;
  //   }
  // }

  
  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  
    // read the acceleration and gyroscope data
    // print the data in CSV format
    Serial.print(accelX, 3);
    Serial.print(',');
    Serial.print(accelY, 3);
    Serial.print(',');
    Serial.print(accelZ, 3);
    Serial.print(',');
    Serial.print(gyroX-gyroDriftX, 3);
    Serial.print(',');
    Serial.print(gyroY-gyroDriftY, 3);
    Serial.print(',');
    Serial.print(gyroZ-gyroDriftZ, 3);
    Serial.println("");

    samplesRead++;
    if (samplesRead == numSamples) {
      // add an empty line if it's the last sample
      Serial.println();
      break;
    } 
  }
  digitalWrite(LED_BLUE, LOW);
}