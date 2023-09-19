#include <LSM6DS3.h>
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

const float accelerationThreshold = 2.5;  // threshold of significant in G's
const int numSamples = 119;

int samplesRead = numSamples;

BLEDis bledis;
BLEHidAdafruit blehid;

LSM6DS3 myIMU(I2C_MODE, 0x6A);
Madgwick filter;  // Madgwick filter
float roll, pitch, yaw;             // attitude
float aX, aY, aZ, gX, gY, gZ;
uint8_t readData;                   // for reading IMU register

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

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  // initialize and set IMU
  // refer to   LSM6D3.cpp:351
  myIMU.settings.gyroRange = 2000;  // calcGyro()
  myIMU.settings.accelRange = 4;    // calcAccel()
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
  filter.begin(12.5);

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

  // BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();
}


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

void readIMU() {
  // wait for IMU data to become valid
  // sample rate is 12.5Hz, so can read every 80mS
  do {
    myIMU.readRegister(&readData, LSM6DS3_ACC_GYRO_STATUS_REG);  //0,0,0,0,0,TDA,GDA,XLDA
  } while ((readData & 0x07) != 0x07);

  // digitalWrite(LED_RED, LOW);   // data read and send task indicator ON

  aX = myIMU.readFloatAccelX();  // Accel data
  aY = myIMU.readFloatAccelY();
  aZ = myIMU.readFloatAccelZ();
  gX = myIMU.readFloatGyroX();  // Gyro data
  gY = myIMU.readFloatGyroY();
  gZ = myIMU.readFloatGyroZ();
}

void storeData() {
  // normalize the IMU data between 0 to 1 and store in the model's
  // input tensor
  tflInputTensor->data.f[samplesRead * 6 + 0] = (aX + 4.0) / 8.0;
  tflInputTensor->data.f[samplesRead * 6 + 1] = (aY + 4.0) / 8.0;
  tflInputTensor->data.f[samplesRead * 6 + 2] = (aZ + 4.0) / 8.0;
  tflInputTensor->data.f[samplesRead * 6 + 3] = (gX + 2000.0) / 4000.0;
  tflInputTensor->data.f[samplesRead * 6 + 4] = (gY + 2000.0) / 4000.0;
  tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ + 2000.0) / 4000.0;
}

void loop() {

  // Serial.println("loop");
  blehid.mouseMove(20, 50);
  readIMU();
  // wait for significant motion
  if (samplesRead == numSamples) {


    // sum up the absolutes
    float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

    // check if it's above the threshold
    if (aSum >= accelerationThreshold) {
      // reset the sample read count
      samplesRead = 0;

    } else {
      return;
    }
  }
  storeData();
        // calculate the attitude with Madgwick filter
      filter.updateIMU(gX, gY, gZ, aX, aY, aZ);

      roll = filter.getRoll();    // -180 ~ 180deg
      pitch = filter.getPitch();  // -180 ~ 180deg
      yaw = filter.getYaw();      // 0 -3 60deg

  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < numSamples) {
    samplesRead++;
    readIMU();
    storeData();
  }

  // Run inferencing
  TfLiteStatus invokeStatus = tflInterpreter->Invoke();
  if (invokeStatus != kTfLiteOk) {
    Serial.println("Invoke failed!");
    while (1)
      ;
    return;
  }

  // Loop through the output tensor values from the model
  for (int i = 0; i < NUM_GESTURES; i++) {
    Serial.print(GESTURES[i]);
    Serial.print(": ");
    Serial.println(tflOutputTensor->data.f[i], 6);
  }
  Serial.println();

}
