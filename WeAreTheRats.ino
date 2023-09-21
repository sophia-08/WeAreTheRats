#include "LSM6DS3.h"
#include <Wire.h>
#include <bluefruit.h>
#include <MadgwickAHRS.h>  // Madgwick 1.2.0 by Arduino

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
//#include <tensorflow/lite/version.h>

#include "model.h"
#define MOUSE_REPORT_ID 1
const float accelerationThreshold = 2.5;  // threshold of significant in G's
const int numSamples = 119;

int samplesRead = numSamples;
#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D7

BLEDis bledis;
// BLEHidGeneric blehid = BLEHidGeneric(1);
BLEHidAdafruit blehid;

LSM6DS3 myIMU(I2C_MODE, 0x6A);
Madgwick filter;                                            // Madgwick filter
float accelX, accelY, accelZ,                               // units m/s/s i.e. accelZ if often 9.8 (gravity)
  gyroX, gyroY, gyroZ,                                      // units dps (degrees per second)
  gyroDriftX, gyroDriftY, gyroDriftZ,                       // units dps
  gyroRoll, gyroPitch, gyroYaw,                             // units degrees (expect major drift)
  gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
  accRoll, accPitch, accYaw,                                // units degrees (roll and pitch noisy, yaw not possible)
  complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;
uint8_t readData;

float roll0, pitch0, yaw0;
float roll, pitch, yaw;
// we use our own descriptor because we want absolute mouse positioning
uint8_t const hid_report_descriptor[] = {
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
  HID_USAGE(HID_USAGE_DESKTOP_MOUSE),
  HID_COLLECTION(HID_COLLECTION_APPLICATION),
  HID_REPORT_ID(1)
    HID_USAGE(HID_USAGE_DESKTOP_POINTER),
  HID_COLLECTION(HID_COLLECTION_PHYSICAL),
  HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),
  HID_USAGE_MIN(1),
  HID_USAGE_MAX(3),
  HID_LOGICAL_MIN(0),
  HID_LOGICAL_MAX(1),
  /* buttons */
  HID_REPORT_COUNT(3),
  HID_REPORT_SIZE(1),
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  /* padding */
  HID_REPORT_COUNT(1),
  HID_REPORT_SIZE(5),
  HID_INPUT(HID_CONSTANT),
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
  /* X, Y position [0, 32767] */
  HID_USAGE(HID_USAGE_DESKTOP_X),
  HID_USAGE(HID_USAGE_DESKTOP_Y),
  HID_LOGICAL_MIN_N(0x0000, 2),
  HID_LOGICAL_MAX_N(0x7fff, 2),
  HID_REPORT_COUNT(2),
  HID_REPORT_SIZE(16),
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  HID_COLLECTION_END,
  HID_COLLECTION_END
};

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* GESTURES[] = {
  "punch",
  "flex"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

//0 ledoff, 1 ledon
int ledgreen = 0;
int ledred = 0;
#define LED_CHARGER 23
void setup() {
  Serial.begin(9600);
  // while (!Serial)
  //   ;

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_CHARGER, OUTPUT);


  pinMode(D7, INPUT_PULLUP);
  pinMode(D8, INPUT_PULLUP);
  pinMode(D9, INPUT_PULLUP);
  pinMode(D10, INPUT_PULLUP);

  digitalWrite(D7, HIGH);
  digitalWrite(D8, HIGH);
  digitalWrite(D9, HIGH);
  digitalWrite(D10, HIGH);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_CHARGER, LOW);

#if 1
  if (myIMU.begin() != 0) {
    Serial.println("IMU Device error");
    while (1)
      ;
  }
  Wire1.setClock(400000UL);  //SCL 400kHz

  // change defalt settings, refer to data sheet 9.13, 9.14, 9.19, 9.20
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x1C);   // 12.5Hz 2000dps
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x1A);  // 12.5Hz 4G
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00);   // HPF 16mHz
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x09);  // ODR/4

  // Maadgwick filter sampling rate
  filter.begin(416);

  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1)
      ;
  }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

  Bluefruit.begin();
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16);  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  Bluefruit.setTxPower(4);                  // Check bluefruit.h for supported values
  Bluefruit.setName("WeAreTheRats");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // blehid.setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));
  // uint16_t input_len[] = { 5 };
  // blehid.setReportLen(input_len, NULL, NULL);
  // BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();

  calibrateIMU(250, 250);
  lastTime = micros();
 #endif 
}


/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int calibrationMillis) {
  gyroDriftX = 0.67;
  gyroDriftY = -1.38;
  gyroDriftZ = -0.51;
  return;

  int calibrationCount = 0;

  delay(delayMillis);  // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;
      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

  // {
  //   Serial.print(calibrationCount);
  //   Serial.print(",");
  //   Serial.print(gyroDriftX);
  //   Serial.print(",");
  //   Serial.print(gyroDriftY);
  //   Serial.print(",");
  //   Serial.print(gyroDriftZ);
  //   Serial.println("");
  // }

  // uint8_t* data = myIMU.dumpRegisters();
  // for (int i = 0; i < 15; i++) {
  //   Serial.print(i);
  //   Serial.print(",");
  //   Serial.println(data[i], HEX);
  // }
  // while (1)
  //   ;
}

/**
   Read accel and gyro data.
   returns true if value is 'new' and false if IMU is returning old cached data
*/
// bool readIMU() {
//   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
//     IMU.readAcceleration(accelX, accelY, accelZ);
//     IMU.readGyroscope(gyroX, gyroY, gyroZ);
//     return true;
//   }
//   return false;
// }

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_GENERIC_HID);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for 'Name' in the advertising packet
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

bool readIMU() {
  // wait for IMU data to become valid
  // sample rate is 416Hz
  // do {
  //   myIMU.readRegister(&readData, LSM6DS3_ACC_GYRO_STATUS_REG);  //0,0,0,0,0,TDA,GDA,XLDA
  // } while ((readData & 0x07) != 0x07);

  // digitalWrite(LED_RED, LOW);   // data read and send task indicator ON

  uint8_t data[12];
  myIMU.readRegisterRegion(data, 0x22, 12);
  int16_t* p = (int16_t*)data;
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
  p++;
  // Serial.print(data[0],HEX);Serial.print(",");Serial.print(data[1],HEX);Serial.print(",");Serial.println(gyroX);
  // Serial.print(data[6],HEX);Serial.print(",");Serial.print(data[7],HEX);Serial.print(",");Serial.println(accelX);
  //   accelX = myIMU.readFloatAccelX();  // Accel data
  //   accelY = myIMU.readFloatAccelY();
  //   accelZ = myIMU.readFloatAccelZ();
  //   gyroX = myIMU.readFloatGyroX();  // Gyro data
  //   gyroY = myIMU.readFloatGyroY();
  //  gyroZ = myIMU.readFloatGyroZ();
  return true;
}

void storeData() {
  // normalize the IMU data between 0 to 1 and store in the model's
  // input tensor
  tflInputTensor->data.f[samplesRead * 6 + 0] = (accelX + 4.0) / 8.0;
  tflInputTensor->data.f[samplesRead * 6 + 1] = (accelY + 4.0) / 8.0;
  tflInputTensor->data.f[samplesRead * 6 + 2] = (accelZ + 4.0) / 8.0;
  tflInputTensor->data.f[samplesRead * 6 + 3] = (gyroX + 2000.0) / 4000.0;
  tflInputTensor->data.f[samplesRead * 6 + 4] = (gyroY + 2000.0) / 4000.0;
  tflInputTensor->data.f[samplesRead * 6 + 5] = (gyroZ + 2000.0) / 4000.0;
}

int count = 0;
// void mousePosition(int16_t x, int16_t y) {
//   uint8_t report[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
//   report[0] = 0;
//   report[1] = x & 0xff;
//   report[2] = (x >> 8) & 0xff;
//   report[3] = y & 0xff;
//   report[4] = (y >> 8) & 0xff;
//   blehid.inputReport(MOUSE_REPORT_ID, report, sizeof(report));
// }

int tmp = 0;
#define report_freq 8
int lastx, lasty;
void loop() {
  // ledred = !ledred;
  //  digitalWrite(LED_RED, ledred);
  digitalWrite(LED_BLUE, digitalRead(D7));
  digitalWrite(LED_RED, digitalRead(D8));
  digitalWrite(LED_GREEN, digitalRead(D9));
  digitalWrite(LED_CHARGER, digitalRead(D10));

  if (!Bluefruit.connected()) return;
  myIMU.readRegister(&readData, LSM6DS3_ACC_GYRO_STATUS_REG);  //0,0,0,0,0,TDA,GDA,XLDA
  if ((readData & 0x07) != 0x07) return;
  // every 2.4ms
  // ledgreen = !ledgreen;
  ledgreen = (ledgreen + 1) % 400;
  digitalWrite(LED_GREEN, ledgreen);
  // Serial.println("loop");
  // blehid.mouseMove(20, 50);

  if (count == report_freq - 1) {
    roll0 = roll;
    pitch0 = pitch;
    yaw0 = yaw;
  }
  count++;
  readIMU();

  // long currentTime = micros();
  // lastInterval = currentTime - lastTime;  // expecting this to be ~104Hz +- 4%
  // lastTime = currentTime;


  doCalculations();

  // wait for significant motion
  // if (samplesRead == numSamples) {


  //   // sum up the absolutes
  //   float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

  //   // check if it's above the threshold
  //   if (aSum >= accelerationThreshold) {
  //     // reset the sample read count
  //     samplesRead = 0;

  //   } else {
  //     return;
  //   }
  // }
  // storeData();
  // calculate the attitude with Madgwick filter
  filter.updateIMU(gyroX - gyroDriftX, gyroY - gyroDriftY, gyroZ - gyroDriftZ, accelX, accelY, accelZ);

  // roll = filter.getRoll();    // -180 ~ 180deg
  // pitch = filter.getPitch();  // -180 ~ 180deg
  // yaw = filter.getYaw();      // 0 - 360deg
  roll = complementaryRoll;    // -180 ~ 180deg
  pitch = complementaryPitch;  // -180 ~ 180deg
  yaw = complementaryYaw;      // 0 - 360deg

  // printCalculations();
  // return;
  int32_t x;
  int32_t y;
#define SMOOTHING_RATIO 0.8
#define SENSITIVITY 50
#define VERTICAL_SENSITIVITY_MULTIPLIER 1.5
  if (count % report_freq == 0) {
    // x = SMOOTHING_RATIO * x + (1 - SMOOTHING_RATIO) * (16384 + -(yaw - yaw0) * SENSITIVITY);
    // x = x - (yaw - yaw0) * SENSITIVITY;
    x = (yaw - yaw0) * SENSITIVITY;
    // x = max(0, min(32767, x));
    // y = y + (pitch - pitch0) * SENSITIVITY;
    // y = SMOOTHING_RATIO * y + (1 - SMOOTHING_RATIO) * (16384 + -(pitch - pitch0) * SENSITIVITY * VERTICAL_SENSITIVITY_MULTIPLIER);
    y = (pitch - pitch0) * SENSITIVITY * VERTICAL_SENSITIVITY_MULTIPLIER;
    // y = max(0, min(32767, y));
    // x = tmp;
    // y = tmp;
    // tmp += 20;
    // tmp = tmp % 32768;
    if (abs(x) > 8 || abs(y) > 8) {
      // mousePosition(x, y);
      Serial.print(-x);Serial.print(",");Serial.println(y);
      blehid.mouseMove(-x, y);
      yaw0 = yaw;
      pitch0 = pitch;
    }


    // yaw0 = yaw;
    // pitch0 = pitch;

    // Serial.print(yaw);
    // Serial.print(",");
    // Serial.print(yaw0);
    // Serial.print(",  \t");
    // Serial.print(pitch);
    // Serial.print(",");
    // Serial.print(pitch0);
    // Serial.print(",  \t");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.println(y);
  }

  // Serial.print(roll);Serial.print(",");
  // Serial.print(pitch);Serial.print(",");
  // Serial.println(yaw);
  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  // while (samplesRead < numSamples) {
  //   samplesRead++;
  //   readIMU();
  //   storeData();
  // }

  // // Run inferencing
  // TfLiteStatus invokeStatus = tflInterpreter->Invoke();
  // if (invokeStatus != kTfLiteOk) {
  //   Serial.println("Invoke failed!");
  //   while (1)
  //     ;
  //   return;
  // }

  // // Loop through the output tensor values from the model
  // for (int i = 0; i < NUM_GESTURES; i++) {
  //   Serial.print(GESTURES[i]);
  //   Serial.print(": ");
  //   Serial.println(tflOutputTensor->data.f[i], 6);
  // }
  // Serial.println();
}

/**
   I'm expecting, over time, the Arduino_LSM6DS3.h will add functions to do most of this,
   but as of 1.0.0 this was missing.
*/
void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

  float lastFrequency = 416;  //(float)1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
}

/**
   This comma separated format is best 'viewed' using 'serial plotter' or processing.org client (see ./processing/RollPitchYaw3d.pde example)
*/
void printCalculations() {
  Serial.print(roll);
  Serial.print(',');
  Serial.print(pitch);
  Serial.print(',');
  Serial.print(yaw);
  Serial.print(',');
  Serial.print(gyroCorrectedRoll);
  Serial.print(',');
  Serial.print(gyroCorrectedPitch);
  Serial.print(',');
  Serial.print(gyroCorrectedYaw);
  Serial.print(',');
  Serial.print(accRoll);
  Serial.print(',');
  Serial.print(accPitch);
  Serial.print(',');
  Serial.print(accYaw);
  Serial.print(',');
  Serial.print(complementaryRoll);
  Serial.print(',');
  Serial.print(complementaryPitch);
  Serial.print(',');
  Serial.print(complementaryYaw);
  Serial.println("");
}
