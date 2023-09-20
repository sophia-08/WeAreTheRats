#include "LSM6DS3.h"
#include <Wire.h>

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A
float accelX, accelY, accelZ,  // units m/s/s i.e. accelZ if often 9.8 (gravity)
    gyroX, gyroY, gyroZ;       // units dps (degrees per second)

const float accelerationThreshold = 1.8; // threshold of significant in G's
const int numSamples = 119;
int samplesRead = numSamples;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    ;
  // Call .begin() to configure the IMUs
  if (myIMU.begin() != 0)
  {
    Serial.println("Device error");
    while (1)
      ;
  }
  Wire1.setClock(400000UL); // SCL 400kHz
}
bool readIMU()
{
  uint8_t readData;
  // wait for IMU data to become valid
  // sample rate is 416Hz
  do
  {
    myIMU.readRegister(&readData, LSM6DS3_ACC_GYRO_STATUS_REG); // 0,0,0,0,0,TDA,GDA,XLDA
  } while ((readData & 0x07) != 0x07);

  // digitalWrite(LED_RED, LOW);   // data read and send task indicator ON

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
void loop()
{
  readIMU();
  // wait for significant motion
  if (samplesRead == numSamples)
  {
    // sum up the absolutes
    float aSum = fabs(accelX) + fabs(accelY) + fabs(accelZ);

    // check if it's above the threshold
    if (aSum < accelerationThreshold)
    {
      return;
    }
  }

  samplesRead = 0;
  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < numSamples)
  {
    // check if both new acceleration and gyroscope data is
    // available
    // read the acceleration and gyroscope data
    // print the data in CSV format
    Serial.print(accelX, 3);
    Serial.print(',');
    Serial.print(accelY, 3);
    Serial.print(',');
    Serial.print(accelZ, 3);
    Serial.print(',');
    // Serial.print(gyroX, 3);
    // Serial.print(',');
    // Serial.print(gyroY, 3);
    // Serial.print(',');
    // Serial.print(gyroZ, 3);
    Serial.println("");

    samplesRead++;
    if (samplesRead == numSamples)
    {
      // add an empty line if it's the last sample
      Serial.println();
    }
    else
    {
      readIMU();
    }
  }
}