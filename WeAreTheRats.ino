// #include "LSM6DS3.h"
#include "Adafruit_BNO055.h"
// #include <Adafruit_Sensor.h>
#include <Wire.h>

#include <bluefruit.h>
// #include <MadgwickAHRS.h>  // Madgwick 1.2.0 by Arduino

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "model.h"
// #define TOM
const float accelerationThreshold = 2.5; // threshold of significant in G's

const int numSamples = 500; // 119;
double samples[numSamples][6];

int samplesRead = 0;
#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D6
#define IMU_RESET D0
#define SWITCH_DEVICE_MODE D10
#define DEBUG_2 D2
#define DEBUG_3 D3

#define out_samples 100

#define SMOOTHING_RATIO 0.8
#define SENSITIVITY_X 30
#define SENSITIVITY_Y 35

#define DEVICE_MOUSE_MODE 0
#define DEVICE_KEYBOARD_MODE 1
int deviceMode;

BLEDis bledis;
BLEHidAdafruit blehid;

#ifdef TOM
// Central uart client
BLEClientUart clientUart;
#endif

float accelX, accelY, accelZ, // units m/s/s i.e. accelZ if often 9.8 (gravity)
    gyroX, gyroY, gyroZ,      // units dps (degrees per second)
    gyroDriftX, gyroDriftY, gyroDriftZ, // units dps
    gyroRoll, gyroPitch, gyroYaw,       // units degrees (expect major drift)
    gyroCorrectedRoll, gyroCorrectedPitch,
    gyroCorrectedYaw, // units degrees (expect minor drift)
    accRoll, accPitch,
    accYaw, // units degrees (roll and pitch noisy, yaw not possible)
    complementaryRoll, complementaryPitch,
    complementaryYaw; // units degrees (excellent roll, pitch, yaw minor drift)

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

const tflite::Model *tflModel = nullptr;
tflite::MicroInterpreter *tflInterpreter = nullptr;
TfLiteTensor *tflInputTensor = nullptr;
TfLiteTensor *tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 128 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char *GESTURES = "abcdefghijklmnopqrstuvwxyz";

#define NUM_GESTURES 26

int ledgreen = 0;
int ledred = 0;
#define LED_CHARGER 23
#define LIGHT_ON LOW
#define LIGHT_OFF HIGH

// const uint8_t BLEUART_UUID_SERVICE[] =
// {
//     0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
//     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
// };

void setup() {
  Serial.begin(115200);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_CHARGER, OUTPUT);
  pinMode(IMU_RESET, OUTPUT);
  pinMode(DEBUG_2, OUTPUT);
  pinMode(DEBUG_3, OUTPUT);

  // Mystery of why !Serial not ready:
  // The "Serial" is always valid for an Arduino Uno, therefor that piece of
  // code does not wait. In the Leonardo, the "Serial" could be zero, if the
  // serial monitor has not been opened yet.

  // while (!Serial) {
  //   digitalWrite(LED_RED, LIGHT_ON);
  //   delay(10);
  //   digitalWrite(LED_RED, LIGHT_OFF);
  //   delay(100);
  // }

  Serial.println("We are the rats\n");
  Serial.println("-------------------------------------\n");

  // Reset IMU
  digitalWrite(IMU_RESET, HIGH);
  delay(0.1);
  digitalWrite(IMU_RESET, LOW);
  delay(0.1);
  digitalWrite(IMU_RESET, HIGH);

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
  digitalWrite(LED_CHARGER, LIGHT_OFF);

  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    systemHaltWithledPattern(LED_RED, 1);
  }

  // Create an interpreter to run the model
  tflInterpreter =
      new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena,
                                   tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  if (tflInterpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors failed!");
    systemHaltWithledPattern(LED_RED, 2);
  };

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

// Initialize Bluefruit with max concurrent connections as Peripheral = 1,
// Central = 1. SRAM usage required by SoftDevice will increase with number of
// connections
#ifdef TOM
  Bluefruit.begin(1, 1);
  Bluefruit.Central.setConnInterval(9, 16);
  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(cent_connect_callback);
  Bluefruit.Central.setDisconnectCallback(cent_disconnect_callback);

#else
  Bluefruit.begin();
#endif
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16);
  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms

  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("WeAreTheRats");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();
  blehid.begin();

#ifdef TOM
  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(cent_bleuart_rx_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0); // 0 = Don't stop scanning after n seconds
#endif
  // Set up and start advertising
  startAdv();

  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    systemHaltWithledPattern(LED_RED, 3);
  }

  // calibrateIMU(250, 250);
  lastTime = micros();
  deviceMode = DEVICE_MOUSE_MODE;
  Serial.print("bno mode ");
  Serial.println(bno.getMode());
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
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0); // 0 = Don't stop advertising after n seconds
}

float minAccl = 10;
float minGyro = 10;
float maxAccl = -10;
float maxGyro = -10;
float rangeOfAccl, rangeOfGyro;
int tensorIndex = 0;

int count = 0;
int tmp = 0;

#define report_freq 2

int lastx, lasty;
int left, right;
int last_left, last_right;

bool inference_started = false;

#define PRECISION 4
float lastAx, lastAy, lastAz;
float lastHeading, lastRoll;
bool startedChar = false;
int t1 = 0;
int ledCount;
bool needSendKeyRelease = false;
float xAngle, yAngle, lastXAngle, lastYAngle;

bool d2;
sensors_event_t orientationData, linearAccelData, angVelData, magneticData;

bool readIMU() {

  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  return true;
}

bool readIMUOrientation1() {
  int loop = 0;
  d2 = !d2;
  digitalWrite(DEBUG_2, d2);
  // Wait upto 25*0.5 ms. The IMU was configured to 100Hz, so shall has new data
  // every 10ms
  while (loop <= 25) {
    loop++;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    if (orientationData.orientation.heading == lastHeading &&
        orientationData.orientation.roll == lastRoll) {
      delay(0.5);
    } else {
      break;
    }
  }

  lastHeading = orientationData.orientation.heading;
  lastRoll = orientationData.orientation.roll;
  return true;
}

bool readIMUOrientation() {
  int loop = 0;
  d2 = !d2;
  digitalWrite(DEBUG_2, d2);
  // Wait upto 25*0.5 ms. The IMU was configured to 100Hz, so shall has new data
  // every 10ms

  // while (loop <= 25) {
  //    loop++;
  //    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  //    if (linearAccelData.acceleration.x == lastAx &&
  //    linearAccelData.acceleration.y == lastAy &&
  //    linearAccelData.acceleration.z == lastAz) {
  //      delay(0.5);
  //    } else {
  //      lastAx = linearAccelData.acceleration.x;
  //      lastAy = linearAccelData.acceleration.y;
  //      lastAz = linearAccelData.acceleration.z;
  //      break;
  //    }
  //  }
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&magneticData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  return true;
}

/**
 * @brief
 *
 * @note expect sample input data in sequences of accelX, accelY, accelZ, gyroX,
 * gyroY, gyroZ
 */
void preprocessData() {
  int pointToRemove;
  float decimate;
  float accumulated = 0.0;
  int removed = 0;
  int start = 0;
  int end = samplesRead - 1;
  // Trim at front. if 2 out of 3 samples have total absolute accelerate read
  // greater than threshold, make the start
  while (true) {
    int count = 0;
    for (int i = 0; i < 3; i++) {
      if (abs(samples[i + start][0]) + abs(samples[i + start][1]) +
              abs(samples[i + start][2]) >
          accelerationThreshold) {
        count += 1;
      }
    }
    if (count >= 2) {
      break;
    } else {
      start += 1;
    }
  }

  // Trim at end,if 2 out of 3 samples have total absolute accelerate read
  // greater than threshold, make the start
  while (true) {
    int count = 0;
    for (int i = 0; i < 3; i++) {
      if (abs(samples[end - i][0]) + abs(samples[end - i][1]) +
              abs(samples[end - i][2]) >
          3) {
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
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][0] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][1] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][2] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][3] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][4] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][5] - minGyro) / rangeOfGyro;
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
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][0] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][1] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][2] - minAccl) / rangeOfAccl;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][3] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][4] - minGyro) / rangeOfGyro;
      tflInputTensor->data.f[tensorIndex++] =
          (samples[i][5] - minGyro) / rangeOfGyro;
    }
  }

  Serial.print("Samples:");
  Serial.print(samplesRead);
  Serial.print(", tensor ");
  Serial.println(tensorIndex);

  // dumpTensors();
}

// void dumpTensors() {
//   for (int i = 0; i < tensorIndex;) {
//     Serial.print(tflInputTensor->data.f[i++]);
//     Serial.print(", ");
//     Serial.print(tflInputTensor->data.f[i++]);
//     Serial.print(", ");
//     Serial.print(tflInputTensor->data.f[i++]);
//     Serial.print(", ");
//     Serial.print(tflInputTensor->data.f[i++]);
//     Serial.print(", ");
//     Serial.print(tflInputTensor->data.f[i++]);
//     Serial.print(", ");
//     Serial.println(tflInputTensor->data.f[i++]);
//   }
// }
// void loadTest() {

//   for (int i = 0; i < numSamples * 6; i++) {
//     tflInputTensor->data.f[i] = tt[i];
//   }
// }

/**
 * @brief Use the given led indicate system halt. the led shall blink at
 * frequence of give duration
 *
 * @param led
 * @param seconds
 */
void systemHaltWithledPattern(int led, int seconds) {

  for (;;) {

    digitalWrite(led, LIGHT_ON);
    delay(1000 * seconds);

    digitalWrite(led, LIGHT_OFF);
    delay(1000 * seconds);
  }
}

int lastSent;
int currentSent;

void loop() {
  // ledred = !ledred;
  //  digitalWrite(LED_RED, ledred);
  // digitalWrite(LED_BLUE, digitalRead(D7));
  // digitalWrite(LED_RED, digitalRead(MOUSE_RIGHT));
  // digitalWrite(LED_GREEN, digitalRead(D9));
  // digitalWrite(LED_CHARGER, digitalRead(D10));
  ledCount++;
  // pluse the green led to indicate system alive.
  if (ledCount % 5000 < 50) {
    digitalWrite(LED_GREEN, LIGHT_ON);
  } else {
    digitalWrite(LED_GREEN, LIGHT_OFF);
  }

  // Press SWITCH_DEVICE_MODE, the read is low
  if (digitalRead(SWITCH_DEVICE_MODE) == LOW) {
    if (deviceMode == DEVICE_MOUSE_MODE) {
      deviceMode = DEVICE_KEYBOARD_MODE;
      Serial.println("swithc to keyboard");
    } else {
      deviceMode = DEVICE_MOUSE_MODE;
      Serial.println("swithc to mouse");
    }
    // wait until key is released.
    while (digitalRead(SWITCH_DEVICE_MODE) == LOW) {
      ;
    };
  }

  if (deviceMode == DEVICE_MOUSE_MODE) {

    // Process the Left and Right click
    left = digitalRead(MOUSE_LEFT);
    right = digitalRead(MOUSE_RIGHT);

    // detect edge
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

    // In mouse mode, we only need orientation.
    readIMUOrientation();

    // In sense fuse mdoe, The IMU runs at 100Hz, which means new data every
    // 10ms. The loop() runs about every 4ms. It's not meanful to process the
    // IMU data if it's stale data. So here we check and only continue after new
    // IMU data is available. Here we also assume for each sample, the IMU read
    // changes.
    if (lastAx == orientationData.orientation.roll &&
        lastAy == orientationData.orientation.pitch) {
      return;
    } else {
      lastAx = orientationData.orientation.roll;
      lastAy = orientationData.orientation.pitch;
    }

    // The below code shall run at interval of 10ms

    currentSent = millis();
    Serial.println(currentSent - lastSent);
    lastSent = currentSent;

    // With current hardware setup:
    // pitch map to vertical movement (y). from top to bottom, y decrease 90 -
    // -90 roll map to horizontal movement (x). from left to right, x increase.
    // 0 - 360

    // Serial.print(orientationData.orientation.pitch);
    // Serial.print(",");
    // // Serial.print(",");
    // Serial.print(orientationData.orientation.roll);
    // Serial.print(",");
    // Serial.print(orientationData.orientation.heading);
    // Serial.print(",");
    // Serial.print(magneticData.magnetic.x);
    // Serial.print(",");
    // // Serial.print(",");
    // Serial.print(magneticData.magnetic.y);
    // Serial.print(",");
    // Serial.println(magneticData.magnetic.z);
    // // Serial.print(",");

    // return;

    xAngle = orientationData.orientation.roll;
    yAngle = orientationData.orientation.pitch;

    if (count == report_freq - 1) {
      lastXAngle = xAngle;
      lastYAngle = yAngle;
    }
    count++;
    int32_t x;
    int32_t y;
    digitalWrite(DEBUG_3, HIGH);

    // We do not want to overload the BLE link.
    // so here we send 1 report every (report_freq * 10ms)
    if (count % report_freq == 0) {
      x = (xAngle - lastXAngle) * SENSITIVITY_X;

      // xAngle go back to 0 after pass 360 degrees. so here we need add the
      // offsets.
      if (x < -180 * SENSITIVITY_X) {
        x += 360 * SENSITIVITY_X;
      }

      y = (yAngle - lastYAngle) * SENSITIVITY_Y;

      // get rid of movement due to noise.
      if (abs(x) > 5 || abs(y) > 5) {
        // if (abs(accelX) + abs(accelY) + abs(accelZ) > 1.5) {
        // mousePosition(x, y);
        // if (abs(x) < 10) x = 0;
        // if (abs(y) < 10) y = 0;
        // Serial.print(x);
        // Serial.print(",");
        // Serial.print(-y);
        // Serial.print(orientationData.orientation.pitch);
        // // Serial.print(",");
        // Serial.print(",  ");
        // Serial.print(orientationData.orientation.roll);
        // Serial.print(",");
        // Serial.println(orientationData.orientation.pitch);
        // Serial.print(",");

        if (digitalRead(MOUSE_ACTIVATE) == HIGH) {
          blehid.mouseMove(x, -y);
        }
        lastXAngle = xAngle;
        lastYAngle = yAngle;
      }
    }
    digitalWrite(DEBUG_3, LOW);

    return;
  }

  /*****  Below is for Keyboard  ******/

  // Device in Keyboard mode

  // When a key is pressed, tow events shall be generated, KEY_UP and KEY_DOWN.
  // When a character is recoganized, a KEY_DOWN event is sent. Here I send the
  // KEY_UP event
  if (needSendKeyRelease) {
    needSendKeyRelease = false;
    blehid.keyRelease();
  }

  // Capture has not started, ignore until user activate keypad
  if (!startedChar) {
    if (digitalRead(MOUSE_ACTIVATE) == LOW) {
      return;
    } else {
      // User activate keypad, check whether 2s passed since last capture
      // int currentTime = millis();
      // if (currentTime < t1 + 2000) {
      //   return;
      // }
      // t1 = currentTime;      
      startedChar = true;
      samplesRead = 0;
      minAccl = 10;
      minGyro = 10;
      maxAccl = -10;
      maxGyro = -10;
    }
  }

  digitalWrite(LED_BLUE, LIGHT_ON);

  // Keep sampling until user release the ACTIVATE button
  while (true) {
  wait:
    // User deactivated keypad
    if (digitalRead(MOUSE_ACTIVATE) == LOW) {
      startedChar = false;
      inference_started = true;
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
      d2 = !d2;
      digitalWrite(DEBUG_2, d2);
    }

    // In case user hold the ACTIVATE button too long
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

    // Invoke ML inference
    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
    if (invokeStatus != kTfLiteOk) {
      Serial.println("Invoke failed!");
    }

    // Loop through the output tensor values from the model
    // for (int i = 0; i < NUM_GESTURES; i++) {
    //   Serial.print(GESTURES[i]);
    //   Serial.print(": ");
    //   Serial.println(tflOutputTensor->data.f[i], 6);
    // }
    // Serial.println();

    char ch = '.';
    for (int i = 0; i < NUM_GESTURES; i++) {
      if (tflOutputTensor->data.f[i] > 0.5) {
        ch = GESTURES[i];
        break;
      };
    }
    Serial.println(ch);

    // Send KEY_DOWN
    blehid.keyPress(ch);

    // Send KEY_UP at next loop
    needSendKeyRelease = true;

#if 1
    // ledgreen = !ledgreen;
    // ledgreen = (ledgreen + 1) % 400;
    // digitalWrite(LED_GREEN, ledgreen);
    // Serial.println("loop");
    // blehid.mouseMove(20, 50);

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
#endif
}

#ifdef TOM

/*------------------------------------------------------------------*/
/* Central
 *------------------------------------------------------------------*/
void scan_callback(ble_gap_evt_adv_report_t *report) {
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised
  // Connect to the device with bleuart service in advertising packet
  Bluefruit.Central.connect(report);
}

void cent_connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = {0};
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Cent] Connected to ");
  Serial.println(peer_name);
  ;

  if (clientUart.discover(conn_handle)) {
    // Enable TXD's notify
    clientUart.enableTXD();
  } else {
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }
}

void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  Serial.println("[Cent] Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param cent_uart Reference object to the service where the data
 * arrived. In this example it is clientUart
 */
void cent_bleuart_rx_callback(BLEClientUart &cent_uart) {
  char str[20 + 1] = {0};
  cent_uart.read(str, 20);

  Serial.print("[Cent] RX: ");
  Serial.println(str);

  // blehid.keyPress(str[0]);
  // needSendKeyRelease = true;

  if (str[0] == 'a') {
    if (deviceMode == DEVICE_MOUSE_MODE) {
      deviceMode = DEVICE_KEYBOARD_MODE;
      Serial.println("swithc to keyboard");
    } else {
      deviceMode = DEVICE_MOUSE_MODE;
      Serial.println("swithc to mouse");
    }
  }
}

#endif
