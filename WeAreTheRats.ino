// #include "LSM6DS3.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"

#include <bluefruit.h>
// #include <MadgwickAHRS.h>  // Madgwick 1.2.0 by Arduino

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
//#include <tensorflow/lite/version.h>

#include "model.h"
// #include "test1.h"

// #define MOUSE_REPORT_ID 1
// const float accelerationThreshold = 2.5;  // threshold of significant in G's

const int numSamples = 500;  //119;
double samples[numSamples][6];

int samplesRead = 0;
#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D6

#define out_samples 120

BLEDis bledis;
BLEHidAdafruit blehid;

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

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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
constexpr int tensorArenaSize = 16 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* GESTURES[] = {
  "a",
  "b",
  "c",
  "d",
  "e",
  "f",
  "g"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

//0 ledoff, 1 ledon
int ledgreen = 0;
int ledred = 0;
#define LED_CHARGER 23
#define LIGHT_ON LOW
#define LIGHT_OFF HIGH
void setup() {
  Serial.begin(115200);
  // while (!Serial)
  //   ;

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_CHARGER, OUTPUT);

  pinMode(MOUSE_ACTIVATE, INPUT_PULLUP);
  pinMode(D7, INPUT_PULLUP);
  pinMode(MOUSE_RIGHT, INPUT_PULLUP);
  pinMode(MOUSE_LEFT, INPUT_PULLUP);
  pinMode(D10, INPUT_PULLUP);

  digitalWrite(MOUSE_ACTIVATE, HIGH);
  digitalWrite(D7, HIGH);
  digitalWrite(MOUSE_RIGHT, HIGH);
  digitalWrite(MOUSE_LEFT, HIGH);
  digitalWrite(D10, HIGH);

  digitalWrite(LED_RED, LIGHT_OFF);
  digitalWrite(LED_BLUE, LIGHT_OFF);
  digitalWrite(LED_GREEN, LIGHT_OFF);
  digitalWrite(LED_CHARGER, LIGHT_OFF);  // HIGH -- LED off.

#if 1
  // if (myIMU.begin() != 0) {
  //   Serial.println("IMU Device error");
  //   while (1)
  //     ;
  // }
  // Wire1.setClock(400000UL);  //SCL 400kHz

  // change defalt settings, refer to data sheet 9.13, 9.14, 9.19, 9.20
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x1C);   // 12.5Hz 2000dps
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x1A);  // 12.5Hz 4G
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00);   // HPF 16mHz
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x09);  // ODR/4

  // Maadgwick filter sampling rate
  // filter.begin(416);

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

#if 0
  Bluefruit.begin();
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16);  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  Bluefruit.setTxPower(4);                  // Check bluefruit.h for supported values
  Bluefruit.setName("WeAreTheRats");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();
  blehid.begin();

  // Set up and start advertising
  startAdv();
#endif
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1)
      ;
  }

  // calibrateIMU(250, 250);
  lastTime = micros();


#endif
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


sensors_event_t orientationData, linearAccelData, angVelData;
bool readIMU() {

  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  return true;
}

float minAccl = 10;
float minGyro = 10;
float maxAccl = -10;
float maxGyro = -10;
float rangeOfAccl, rangeOfGyro;
int tensorIndex = 0;

void preprocessData() {
  int pointToRemove;
  float decimate;
  float accumulated = 0.0;
  int removed = 0;
  int start = 0;
  int end = samplesRead - 1;
  // Trim at front
  while (true) {
    int count = 0;
    for (int i = 0; i < 3; i++) {
      if (abs(samples[i + start][0]) + abs(samples[i + start][1]) + abs(samples[i + start][2]) > 3) {
        count += 1;
      }
    }
    if (count >= 2) {
      break;
    } else {
      start += 1;
    }
  }

  // Trim at end
  while (true) {
    int count = 0;
    for (int i = 0; i < 3; i++) {
      if (abs(samples[end - i][0]) + abs(samples[end - i][1]) + abs(samples[end - i][2]) > 3) {
        count += 1;
      }
    }
    if (count >= 2) {
      break;
    } else {
      end -= 1;
    }
  }

  Serial.print(start);
  Serial.print(" e ");
  Serial.println(end);

  if (end - start + 1 > out_samples) {
    pointToRemove = end - start + 1 - out_samples;
    decimate = float(pointToRemove) / float(out_samples);
    int i = start;
    removed = 0;
    accumulated = 0.0;
    while (true) {
      tflInputTensor->data.f[tensorIndex++] = (samples[i][0] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][1] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][2] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][3] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][4] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][5] - minGyro) / rangeOfGyro;
      accumulated += decimate;
      while (accumulated >= 1) {
        i += 1;
        removed++;
        accumulated -= 1;
      }
      i += 1;
      if (i >= end) {
        break;
      }
    }
    if (removed < pointToRemove) {
      tflInputTensor->data.f[tensorIndex++] = (samples[i][0] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][1] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][2] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][3] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][4] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] = (samples[i][5] - minGyro) / rangeOfGyro;
    }
  }

  Serial.print("Samples:");
  Serial.print(samplesRead);
  Serial.print(", tensor ");
  Serial.println(tensorIndex);

  // dumpTensors();
}

void dumpTensors() {
  for (int i=0; i<tensorIndex; ) {
    Serial.print(tflInputTensor->data.f[i++]);Serial.print(", ");
    Serial.print(tflInputTensor->data.f[i++]);Serial.print(", ");
    Serial.print(tflInputTensor->data.f[i++]);Serial.print(", ");
    Serial.print(tflInputTensor->data.f[i++]);Serial.print(", ");
    Serial.print(tflInputTensor->data.f[i++]);Serial.print(", ");
    Serial.println(tflInputTensor->data.f[i++]);
  }
}
// void loadTest() {

//   for (int i = 0; i < numSamples * 6; i++) {
//     tflInputTensor->data.f[i] = tt[i];
//   }
// }

int count = 0;
int tmp = 0;
#define report_freq 5
int lastx, lasty;
int left, right;
int last_left, last_right;

bool inference_started = false;

#define PRECISION 4
float lastAx, lastAy, lastAz;
bool startedChar = false;
int t1 = 0;
int ledCount;

void loop() {
  // ledred = !ledred;
  //  digitalWrite(LED_RED, ledred);
  // digitalWrite(LED_BLUE, digitalRead(D7));
  // digitalWrite(LED_RED, digitalRead(MOUSE_RIGHT));
  // digitalWrite(LED_GREEN, digitalRead(D9));
  // digitalWrite(LED_CHARGER, digitalRead(D10));
  ledCount++;
  // HIGH  -- LIGHT_OFF
  if (ledCount % 10 == 0) {
    digitalWrite(LED_GREEN, LIGHT_ON);
  } else {
    digitalWrite(LED_GREEN, LIGHT_OFF);
  }

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
      minAccl = 10;
      minGyro = 10;
      maxAccl = -10;
      maxGyro = -10;
    }
  }

  digitalWrite(LED_BLUE, LIGHT_ON);
  while (true) {
wait:
    // User deactivated keypad
    if (digitalRead(MOUSE_ACTIVATE) == LOW) {
      startedChar = false;
      inference_started = true;
      break;
    }
    readIMU();
    if (linearAccelData.acceleration.x == lastAx && linearAccelData.acceleration.y == lastAy && linearAccelData.acceleration.z == lastAz) {
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

      for (int i = 0; i < 3; i++) {
        if (minAccl > samples[samplesRead][i]) {
          minAccl = samples[samplesRead][i];
        }
        if (maxAccl < samples[samplesRead][i]) {
          maxAccl = samples[samplesRead][i];
        }
      }
      for (int i = 3; i < 6; i++) {
        if (minGyro > samples[samplesRead][i]) {
          minGyro = samples[samplesRead][i];
        }
        if (maxGyro < samples[samplesRead][i]) {
          maxGyro = samples[samplesRead][i];
        }
      }
      // Serial.print(samplesRead);
      // Serial.print(',');
      // for (int i = 0; i < 5; i++) {
      //   Serial.print(samples[samplesRead][i], PRECISION);
      //   Serial.print(',');
      // }
      // Serial.println(samples[samplesRead][5], PRECISION);
      samplesRead++;
    }
    if (samplesRead >= numSamples) {
      samplesRead = 0;
    }
  }

  digitalWrite(LED_BLUE, LIGHT_OFF);
  // Not enough samples, restart
  if (samplesRead < out_samples) {
    Serial.print("not enough samples, ");
    Serial.println(samplesRead);
    samplesRead = 0;
    inference_started = false;
    startedChar = false;
    return;
  }

  Serial.print("ranges:");
  Serial.print(minAccl);
  Serial.print(",");
  Serial.print(maxAccl);
  Serial.print(",");
  Serial.print(minGyro);
  Serial.print(",");
  Serial.println(maxGyro);

  if (inference_started) {
    inference_started = false;
    tensorIndex = 0;
    rangeOfAccl = maxAccl - minAccl;
    rangeOfGyro = maxGyro - minGyro;
    preprocessData();

    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
    if (invokeStatus != kTfLiteOk) {
      Serial.println("Invoke failed!");
    }

    // Loop through the output tensor values from the model
    for (int i = 0; i < NUM_GESTURES; i++) {
      Serial.print(GESTURES[i]);
      Serial.print(": ");
      Serial.println(tflOutputTensor->data.f[i], 6);
    }
    Serial.println();

    char ch = '.';
    for (int i = 0; i < NUM_GESTURES; i++) {
      if (tflOutputTensor->data.f[i] > 0.5) {
        ch = GESTURES[i][0];
        break;
      };
    }
    Serial.println(ch);

  }
  return;

#if 0
  left = digitalRead(MOUSE_LEFT);
  right = digitalRead(MOUSE_RIGHT);
  if (left != last_left) {
    if (left == LOW) {
      blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
      Serial.println("left down");
    } else {
      blehid.mouseButtonRelease();
      Serial.println("left up");
    }
    last_left = left;
  }
  if (right != last_right) {
    if (right == LOW) {
      blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
      Serial.println("right down");
    } else {
      blehid.mouseButtonRelease();
      Serial.println("right up");
    }
    last_right = right;
  }

#endif
#if 0
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

  roll = filter.getRoll();    // -180 ~ 180deg
  pitch = filter.getPitch();  // -180 ~ 180deg
  yaw = filter.getYaw();      // 0 - 360deg
  // roll = complementaryRoll;    // -180 ~ 180deg
  // pitch = complementaryPitch;  // -180 ~ 180deg
  // yaw = complementaryYaw;      // 0 - 360deg

  // printCalculations();
  // return;
  int32_t x;
  int32_t y;
#define SMOOTHING_RATIO 0.8
#define SENSITIVITY 60
#define VERTICAL_SENSITIVITY_MULTIPLIER 1.2
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

#if 0
    if (abs(x) > 10 || abs(y) > 10) {
      // if (abs(accelX) + abs(accelY) + abs(accelZ) > 1.5) {
      // mousePosition(x, y);
      Serial.print(-x);
      Serial.print(",");
      Serial.println(y);
      if (digitalRead(MOUSE_ACTIVATE) == HIGH) {
        blehid.mouseMove(-x, y);
      }
      yaw0 = yaw;
      pitch0 = pitch;
    }
#endif
    // {
    //   Serial.print(accelX);
    //   Serial.print(",");
    //   Serial.print(accelY);
    //   Serial.print(",");
    //   Serial.print(accelZ);
    //   Serial.print(",");

    //   Serial.print(-x);
    //   Serial.print(",");
    //   Serial.println(y);
    // }


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
  if (inference_started && samplesRead < numSamples) {
    storeData();
    samplesRead++;
  }

  // // Run inferencing
  if (inference_started && samplesRead == numSamples) {
    samplesRead = 0;
    inference_started = false;

    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
    if (invokeStatus != kTfLiteOk) {
      Serial.println("Invoke failed!");
    }

    // Loop through the output tensor values from the model
    for (int i = 0; i < NUM_GESTURES; i++) {
      Serial.print(GESTURES[i]);
      Serial.print(": ");
      Serial.println(tflOutputTensor->data.f[i], 6);
    }
    Serial.println();
  }
#endif
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
