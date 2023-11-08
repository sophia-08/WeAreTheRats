// #define TOM
// #define BNO055
// #define TSFLOW
#define BNO085

// #include "LSM6DS3.h"
#ifdef BNO055
#include "Adafruit_BNO055.h"
#endif
#ifdef BNO085
#include <Adafruit_BNO08x.h>
#endif
// #include <Adafruit_Sensor.h>
#include <Wire.h>

#include <bluefruit.h>
// #include <MadgwickAHRS.h>  // Madgwick 1.2.0 by Arduino

#ifdef TSFLOW
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#endif
#include "battery.h"
#include "local_constants.h"
#ifdef TSFLOW
#include "model.h"
#endif
#include "system.h"

const float accelerationThreshold = 2.5; // threshold of significant in G's

const int numSamples = 500; // 119;
double samples[numSamples][6];

int samplesRead = 0;
#define out_samples 100

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

#ifdef BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t orientationData, linearAccelData, angVelData, magneticData;
#endif

#ifdef BNO085
#define BNO08X_RESET -1
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ROTATION_VECTOR; // SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 10000;
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(sh2_RotationVectorWAcc_t *rv, euler_t *ypr,
                       bool degrees = false) {

  float qr = rv->real;
  float qi = rv->i;
  float qj = rv->j;
  float qk = rv->k;
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
#endif

#ifdef TSFLOW
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
#endif

// array to map gesture index to a name
const char *GESTURES = "abcdefghijklmnopqrstuvwxyz";

#define NUM_GESTURES 26

int ledgreen = 0;
int ledred = 0;

// const uint8_t BLEUART_UUID_SERVICE[] =
// {
//     0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
//     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
// };
void setup() {
  configGpio();
  Serial.begin(115200);
  // while (!Serial) delay(10);

#ifdef TSFLOW
  loadTFLiteModel();
#endif

  initAndStartBLE();

#ifdef BNO055
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    systemHaltWithledPattern(LED_RED, 3);
  }
  Serial.print("bno mode ");
  Serial.println(bno.getMode());
#endif

#ifdef BNO085
  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
#endif

  // calibrateIMU(250, 250);
  lastTime = micros();
  deviceMode = DEVICE_MOUSE_MODE;
}

float minAccl = 10;
float minGyro = 10;
float maxAccl = -10;
float maxGyro = -10;
float rangeOfAccl, rangeOfGyro;
int tensorIndex = 0;

int count = 0;
int tmp = 0;

#define report_freq 3

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

bool readIMU() {
#ifdef BNO055
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
#endif
  return true;
}

bool readIMUOrientation1() {
#ifdef BNO055
  int loop = 0;
  d2 = !d2;
  // digitalWrite(DEBUG_2, d2);
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
#endif
  return true;
}

bool readIMUOrientation() {
#ifdef BNO055
  int loop = 0;
  d2 = !d2;
  // digitalWrite(DEBUG_2, d2);
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
#endif
  return true;
}

int lastSent;
int currentSent;
int sleepCount;

void loop() {

  ledCount++;
  // pluse the green led to indicate system alive.
  if (ledCount % 1000 < 10) {
    if (deviceMode == DEVICE_MOUSE_MODE) {
      digitalWrite(LED_GREEN, LIGHT_ON);
      digitalWrite(LED_BLUE, LIGHT_OFF);
    } else {
      digitalWrite(LED_BLUE, LIGHT_ON);
      digitalWrite(LED_GREEN, LIGHT_OFF);
    }

  } else {
    if (digitalRead(MOUSE_ACTIVATE) == HIGH) {
      digitalWrite(LED_GREEN, LIGHT_ON);
    } else {
      digitalWrite(LED_GREEN, LIGHT_OFF);
    }
    if (digitalRead(KEYPAD_ACTIVATE) == HIGH) {
      digitalWrite(LED_BLUE, LIGHT_ON);
    } else {
      digitalWrite(LED_BLUE, LIGHT_OFF);
    }
  }

#ifdef BNO085
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE
    // define (above)
    switch (sensorValue.sensorId) {
    // case SH2_ARVR_STABILIZED_RV:
    case SH2_ROTATION_VECTOR:
      quaternionToEuler(&sensorValue.un.rotationVector, &ypr, true);
      break;
      // case SH2_GYRO_INTEGRATED_RV:
      //   // faster (more noise?)
      //   quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
      // break;
    }
    static long last = 0;
    long now = micros();
    // Serial.print(now - last);             Serial.print("\t");
    // last = now;
    // Serial.print(sensorValue.status);     Serial.print("\t");  // This is
    // accuracy in the range of 0 to 3 Serial.print(ypr.yaw);
    // Serial.print("\t"); Serial.print(ypr.pitch); Serial.print("\t");
    // Serial.println(ypr.roll);
    xAngle = -ypr.yaw;
    yAngle = ypr.roll;
  }
  // return;

#endif

  scanNavigateButtons();
  scanClickButtons();

  // When a key is pressed, tow events shall be generated, KEY_UP and KEY_DOWN.
  // For air writing, when a character is recoganized, only KEY_DOWN event is
  // sent. so I need generate a KEY_UP event. needSendKeyRelease is used for the
  // purpose. At the end of air writing, needSendKeyRelease is set.  In next
  // round loop(), here I send the KEY_UP event and reset the flag.
  if (needSendKeyRelease) {
    needSendKeyRelease = false;
    blehid.keyRelease();
  }

#ifdef ENABLE_SLEEP
  if (digitalRead(DEVICE_ACTIVATE) == LOW) {
    sleepCount++;
  } else {
    sleepCount = 0;
  }

  if (sleepCount > 100) {
    // digitalWrite(LED_RED, LIGHT_ON);
    systemSleep();
    // digitalWrite(LED_RED, LIGHT_OFF);
  }
#endif

  if (deviceMode == DEVICE_MOUSE_MODE) {
#ifdef BNO055
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

    // currentSent = millis();
    // Serial.println(currentSent - lastSent);
    // lastSent = currentSent;

    // With current hardware setup:
    // pitch map to vertical movement (y). from top to bottom, y decrease 90 -
    // -90 roll map to horizontal movement (x). from left to right, x increase.
    // 0 - 360

    // Serial.print(orientationData.orientation.pitch);
    // Serial.print(",");
    // Serial.print(orientationData.orientation.roll);
    // Serial.print(",");
    // Serial.println(orientationData.orientation.heading);
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
#endif
    if (count == report_freq - 1) {
      lastXAngle = xAngle;
      lastYAngle = yAngle;
    }
    count++;
    int32_t x;
    int32_t y;
    // digitalWrite(DEBUG_3, HIGH);

    // We do not want to overload the BLE link.
    // so here we send 1 report every (report_freq * 10ms) = 30ms
    if (count % report_freq == 0) {
      x = (xAngle - lastXAngle) * SENSITIVITY_X;

      // xAngle go back to 0 after pass 360 degrees. so here we need add the
      // offsets.
      if (x < -180 * SENSITIVITY_X) {
        x += 360 * SENSITIVITY_X;
      }

      y = (yAngle - lastYAngle) * SENSITIVITY_Y;

      // get rid of movement due to noise.
      if (abs(x) > 8 || abs(y) > 8) {
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
    // digitalWrite(DEBUG_3, LOW);

    return;
  }

  /*****  Below is for Keyboard  ******/

  // Device in Keyboard mode

  // Capture has not started, ignore until user activate keypad
  if (!startedChar) {
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
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
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
      startedChar = false;
      inference_started = true;
      break;
    }
    readIMU();

#ifdef BNO055
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
      // digitalWrite(DEBUG_2, d2);
    }
#endif

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

#ifdef TSFLOW
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
#endif

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

#define DOUBLE_CLICK_INTERVAL 400
#define MOUSE_STEPS_PER_CLICK 5
int lastUpTime, lastDownTime, lastKey;
uint8_t navigateButtons[4] = {KEYPAD_LEFT, KEYPAD_RIGHT, KEYPAD_UP,
                              KEYPAD_DOWN};
uint8_t navigateButtonLastState[4] = {HIGH, HIGH, HIGH, HIGH};
uint8_t navigateButtonInDoubleClickMode[4] = {0, 0, 0, 0};
uint8_t navigateButtonSingleClickKeyboardCode[4] = {
    HID_KEY_ARROW_LEFT, HID_KEY_ARROW_RIGHT, HID_KEY_ARROW_UP,
    HID_KEY_ARROW_DOWN};
uint8_t navigateButtonDoubleClickKeyboardCode[4] = {
    HID_KEY_HOME, HID_KEY_END, HID_KEY_PAGE_UP, HID_KEY_PAGE_DOWN};

int8_t navigateButtonSingleClickMouseCode[4][2] = {{-MOUSE_STEPS_PER_CLICK, 0},
                                                   {MOUSE_STEPS_PER_CLICK, 0},
                                                   {0, -MOUSE_STEPS_PER_CLICK},
                                                   {0, MOUSE_STEPS_PER_CLICK}};
int8_t navigateButtonDoubleClickMouseCode[4] = {MOUSE_BUTTON_BACKWARD,
                                                MOUSE_BUTTON_FORWARD, -1, 1};
uint32_t navigateButtonLastDownTime[4];
uint32_t skipScroll;

uint8_t clickButtons[] = {MOUSE_LEFT, MOUSE_RIGHT, MOUSE_ACTIVATE,
                          KEYPAD_ACTIVATE};
uint8_t clickButtonLastState[] = {HIGH, HIGH, HIGH, HIGH};
uint8_t clickButtonCode[] = {MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT, 0, 0};
uint8_t clickButtonKeyboardCode[] = {HID_KEY_ENTER, HID_KEY_BACKSPACE, 0, 0};
// int lastTimestampScanMouseClick;

void scanOneClickButton(uint8_t keyIndex) {

  uint8_t state = digitalRead(clickButtons[keyIndex]);
  if (state == clickButtonLastState[keyIndex]) { // no change
    return;
  }

  delay(3);
  state = digitalRead(clickButtons[keyIndex]);
  if (state == clickButtonLastState[keyIndex]) { // no change
    return;
  }

  //  edge is detected
  // Serial.println(left);
  clickButtonLastState[keyIndex] = state;

  switch (clickButtons[keyIndex]) {
  case MOUSE_ACTIVATE:
    deviceMode = DEVICE_MOUSE_MODE;
    break;
  case KEYPAD_ACTIVATE:
    deviceMode = DEVICE_KEYBOARD_MODE;
    break;
  default:
    if (deviceMode == DEVICE_MOUSE_MODE) {
      if (state == LOW) {
        blehid.mouseButtonPress(clickButtonCode[keyIndex]);
        Serial.println("button down");
      } else {
        blehid.mouseButtonRelease();
        Serial.println("button up");
      }
    } else {
      if (state == LOW) {
        uint8_t keycodes[6] = {HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE,
                               HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE};
        keycodes[0] = clickButtonKeyboardCode[keyIndex];
        blehid.keyboardReport(0, keycodes);
        Serial.println("button down");
      } else {
        blehid.keyRelease();
        Serial.println("button up");
      }
    }
  }
}

void scanClickButtons() {

  // Only mouse left and right click
  for (int i = 0; i < 4; i++) {
    scanOneClickButton(i);
  }
}

void scanOneNavigateButton(uint8_t keyIndex) {
  // detect edge
  uint8_t state = digitalRead(navigateButtons[keyIndex]);
  if (state == navigateButtonLastState[keyIndex]) { // no change

    // For mouse, when the button is pressed and held,  we need continue send
    // mouseMove() event. This need be done without wait.
    if (state == LOW && deviceMode == DEVICE_MOUSE_MODE) {
      if (navigateButtonInDoubleClickMode[keyIndex]) {
        // Serial.print("mouse scroll: ");
        // Serial.println(navigateButtonDoubleClickMouseCode[keyIndex]);
        // in press and hold mode, Scroll too fast, skip every x
        skipScroll++;
        if (skipScroll % 6 == 0) {
          blehid.mouseScroll(navigateButtonDoubleClickMouseCode[keyIndex]);
        }
      } else {
        // Serial.print("mouse move: ");
        // Serial.print(navigateButtonSingleClickMouseCode[keyIndex][0]);
        // Serial.print(" ");
        // Serial.println(navigateButtonSingleClickMouseCode[keyIndex][1]);
        blehid.mouseMove(navigateButtonSingleClickMouseCode[keyIndex][0],
                         navigateButtonSingleClickMouseCode[keyIndex][1]);
      }
    }
    return;
  };

  delay(1);
  state = digitalRead(navigateButtons[keyIndex]);
  if (state == navigateButtonLastState[keyIndex]) { // only noise
    return;
  }

  navigateButtonLastState[keyIndex] = state;
  uint32_t time1 = millis();
  bool doubleClick = false;

  // high -> low
  if (state == LOW) {
    // If the button was pressed again within threshold, it's a double click
    if (time1 - navigateButtonLastDownTime[keyIndex] < DOUBLE_CLICK_INTERVAL) {
      doubleClick = true;

      // For Mouse. save the double click flag, used for repeat events.
      if (navigateButtons[keyIndex] == KEYPAD_UP ||
          navigateButtons[keyIndex] == KEYPAD_DOWN) {
        navigateButtonInDoubleClickMode[keyIndex] = true;
      }
    }
    if (deviceMode == DEVICE_KEYBOARD_MODE) {
      // keyboard mode
      uint8_t keycodes[6] = {HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE,
                             HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE};
      if (doubleClick) {
        keycodes[0] = navigateButtonDoubleClickKeyboardCode[keyIndex];
      } else {
        keycodes[0] = navigateButtonSingleClickKeyboardCode[keyIndex];
      }

      // Serial.print("key: ");
      // Serial.println(keycodes[0]);
      blehid.keyboardReport(0, keycodes);
    } else {
      // mouse mode
      if (doubleClick) {
        switch (navigateButtons[keyIndex]) {
        case KEYPAD_UP:
        case KEYPAD_DOWN:
          Serial.print("mouse scroll: ");
          Serial.println(navigateButtonDoubleClickMouseCode[keyIndex]);
          blehid.mouseScroll(navigateButtonDoubleClickMouseCode[keyIndex]);
          skipScroll = 0;
          break;
        case KEYPAD_LEFT:
        case KEYPAD_RIGHT:
          blehid.mouseButtonPress(navigateButtonDoubleClickMouseCode[keyIndex]);
          Serial.print("mouse db ");
          Serial.println(navigateButtonDoubleClickMouseCode[keyIndex]);
        }

      } else {
        // Serial.print("mouse move: ");
        // Serial.print(navigateButtonSingleClickMouseCode[keyIndex][0]);
        // Serial.print(" ");
        // Serial.println(navigateButtonSingleClickMouseCode[keyIndex][1]);
        blehid.mouseMove(navigateButtonSingleClickMouseCode[keyIndex][0],
                         navigateButtonSingleClickMouseCode[keyIndex][1]);
      }
    }
    navigateButtonLastDownTime[keyIndex] = time1;
  } else {
    // low -> high

    // Reset double click flag.
    navigateButtonInDoubleClickMode[keyIndex] = false;

    if (deviceMode == DEVICE_KEYBOARD_MODE) {
      // keyboard mode. sent key release event
      // Serial.println("key released");
      blehid.keyRelease();
    } else {
      // mouse mode
      if (doubleClick) {
        switch (navigateButtons[keyIndex]) {
        case KEYPAD_LEFT:
        case KEYPAD_RIGHT:
          blehid.mouseButtonRelease();
          break;
        default:
          break;
        }
      }
    }
  }
}

void scanNavigateButtons() {
  for (int i = 0; i < 4; i++) {
    scanOneNavigateButton(i);
  }
}

void configGpio() {
  // enable battery measuring.
  pinMode(VBAT_ENABLE, OUTPUT);
  // Due to hardware limitation, do not set to high on Seeed nrf52
  digitalWrite(VBAT_ENABLE, LOW);

  // Read charge state. Low is charging.
  pinMode(BAT_CHARGE_STATE, INPUT);

  // Set charge mode. Set to high charging current (100mA)
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(IMU_RESET, OUTPUT);

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

  // Reset IMU
  digitalWrite(IMU_RESET, HIGH);
  delay(0.1);
  digitalWrite(IMU_RESET, LOW);
  delay(0.1);
  digitalWrite(IMU_RESET, HIGH);

  pinMode(MOUSE_ACTIVATE, INPUT_PULLUP);
  pinMode(MOUSE_RIGHT, INPUT_PULLUP);
  pinMode(MOUSE_LEFT, INPUT_PULLUP);
  pinMode(KEYPAD_LEFT, INPUT_PULLUP);
  pinMode(KEYPAD_RIGHT, INPUT_PULLUP);
  pinMode(KEYPAD_CENTER, INPUT_PULLUP);
  pinMode(KEYPAD_UP, INPUT_PULLUP);
  pinMode(KEYPAD_DOWN, INPUT_PULLUP);

  digitalWrite(MOUSE_ACTIVATE, HIGH);
  digitalWrite(MOUSE_RIGHT, HIGH);
  digitalWrite(MOUSE_LEFT, HIGH);
  digitalWrite(KEYPAD_LEFT, HIGH);
  digitalWrite(KEYPAD_RIGHT, HIGH);
  digitalWrite(KEYPAD_CENTER, HIGH);
  digitalWrite(KEYPAD_UP, HIGH);
  digitalWrite(KEYPAD_DOWN, HIGH);

  digitalWrite(LED_RED, LIGHT_OFF);
  digitalWrite(LED_BLUE, LIGHT_OFF);
  digitalWrite(LED_GREEN, LIGHT_OFF);
}

#ifdef TSFLOW
void loadTFLiteModel() {
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
}
#endif

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
void initAndStartBLE() {

#ifdef TOM
  // Initialize Bluefruit with max concurrent connections as Peripheral = 1,
  // Central = 1. SRAM usage required by SoftDevice will increase with number of
  // connections
  Bluefruit.begin(1, 1);
  Bluefruit.Central.setConnInterval(100, 200);
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
  Bluefruit.Scanner.setInterval(160, 5); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0); // 0 = Don't stop scanning after n seconds
#endif
  // Set up and start advertising
  startAdv();
}

#ifdef TSFLOW
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
#endif
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
