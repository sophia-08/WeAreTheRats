#include "local_constants.h"

#include "battery.h"
#include <bluefruit.h>
// #include <MadgwickAHRS.h>  // Madgwick 1.2.0 by Arduino
#include "imu.h"

#ifdef TSFLOW
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#endif

#include "pimoroniTrackball.h"

// Define your custom VID and PID
#define VENDOR_ID 0x3333       // Replace with your Vendor ID
#define PRODUCT_ID 0x5678      // Replace with your Product ID
#define PRODUCT_VERSION 0x0100 // Product version

#ifdef TSFLOW
#include "model.h"
#endif
#include "system.h"
void setBdDAAndName(unsigned char byte3, char *name);

// const float accelerationThreshold = 2.5; // threshold of significant in G's

int samplesRead = 0;
#define out_samples 150
int tensorIndex = 0;
bool noModeSwitch = false;

int deviceMode;
BLEDis bledis;
BLEHidAdafruit blehid;

int deviceId = 0;
unsigned addrByte3;

extern bool newData;

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
constexpr int tensorArenaSize = 160 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));
#endif

// array to map gesture index to a name
const char *GESTURES = "abcdefghijklmnopqrstuvwxyz";

#define NUM_GESTURES 26

void setup() {
  deviceMode = DEVICE_MOUSE_MODE;
  configGpio();
  Serial.begin(115200);
  int i = 0;
  // while (!Serial) {
  //   digitalWrite(LED_RED, LIGHT_ON);
  //   delay(10);
  //   // if (++i > 1000) {
  //   //   break;
  //   // }
  // }
  digitalWrite(LED_RED, LIGHT_OFF);

#ifdef TSFLOW
  loadTFLiteModel();
#endif

  initAndStartBLE();

  imuInit(deviceMode);

  // calibrateIMU(250, 250);

  // nrf_gpio_cfg_sense_input(g_ADigitalPinMap[IMU_INT],
  //                        NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

  //
  // attachInterrupt(IMU_INT, myinthandler, FALLING); // RISING
#ifdef PIMORONI_TRACKBALL
  trackball.begin();
  for (int i = 0; i < 1; i++) {
    trackball.setRed(240);
    trackball.setBlue(0);
    delay(100);
    trackball.setRed(0);
    trackball.setGreen(240);
    delay(100);
    trackball.setGreen(0);
    trackball.setBlue(240);
    delay(100);
  }
  trackball.setBlue(0);
  trackball.setGreen(60);
#endif
}

int count = 0;

bool inference_started = false;

#define PRECISION 4
bool startedChar = false;
int t1 = 0;
int ledCount;
bool needSendKeyRelease = false;
float xAngle, yAngle, lastXAngle, lastYAngle;
int xArrow = 0, yArrow = 0;

void loop() {

  leds();

  scanClickButtons();

#ifdef PIMORONI_TRACKBALL
  if (trackball.changed()) {
    // Bug of the hardware, int keep low.
    // https://github.com/pimoroni/pimoroni-pico/issues/357 if
    // (digitalRead(PIMORONI_TRACKBALL_INT) == LOW) {
    int x, y;
    if (deviceMode == DEVICE_MOUSE_MODE) {
      y = (trackball.right() - trackball.left()) * MINUTE_MOVEMENT;
      x = (-trackball.down() + trackball.up()) * MINUTE_MOVEMENT;
      if (x != 0 || y != 0) {
        // Serial.print(x);
        // Serial.print(",");
        // Serial.println(y);
        blehid.mouseMove(x, y);
      }
      if (trackball.click()) {
        blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
        Serial.println("do");
        noModeSwitch = true;
      }
      if (trackball.release()) {
        blehid.mouseButtonRelease();
        Serial.println("re");
        noModeSwitch = false;
      }
    } else {
      y = (trackball.right() - trackball.left());
      x = (-trackball.down() + trackball.up());
      xArrow += x;
      yArrow += y;

      // keyboard mode
      uint8_t keycodes[6] = {HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE,
                             HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE};

      // scroll up/down
      bool keyPressed = false;
      if (yArrow < -TRACKBALL_ARROW_KEY_SENSIVITY) {
        keycodes[0] = HID_KEY_ARROW_UP;
        keyPressed = true;
        yArrow += TRACKBALL_ARROW_KEY_SENSIVITY;
      } else if (yArrow > TRACKBALL_ARROW_KEY_SENSIVITY) {
        keycodes[0] = HID_KEY_ARROW_DOWN;
        keyPressed = true;
        yArrow -= TRACKBALL_ARROW_KEY_SENSIVITY;
      }

      if (keyPressed) {
        blehid.keyboardReport(0, keycodes);
        delay(1);
        blehid.keyRelease();
      }

      // scroll left/right
      keyPressed = false;
      if (xArrow < -TRACKBALL_ARROW_KEY_SENSIVITY) {
        keycodes[0] = HID_KEY_ARROW_LEFT;
        keyPressed = true;
        xArrow += TRACKBALL_ARROW_KEY_SENSIVITY;
      } else if (xArrow > TRACKBALL_ARROW_KEY_SENSIVITY) {
        keycodes[0] = HID_KEY_ARROW_RIGHT;
        keyPressed = true;
        xArrow -= TRACKBALL_ARROW_KEY_SENSIVITY;
      }

      if (keyPressed) {
        blehid.keyboardReport(0, keycodes);
        delay(1);
        blehid.keyRelease();
      }

      // central button is pressed
      if (trackball.click()) {
        keycodes[0] = HID_KEY_ENTER;
        blehid.keyboardReport(0, keycodes);
        noModeSwitch = true;
      }
      // central button is released
      if (trackball.release()) {
        keycodes[0] = HID_KEY_NONE;
        blehid.keyboardReport(0, keycodes);
        noModeSwitch = false;
      }
    }
  }
#endif

  // When a key is pressed, tow events shall be generated, KEY_UP and KEY_DOWN.
  // For air writing, when a character is recoganized, only KEY_DOWN event is
  // sent. so I need generate a KEY_UP event. needSendKeyRelease is used for the
  // purpose. At the end of air writing, needSendKeyRelease is set.  In next
  // round loop(), here I send the KEY_UP event and reset the flag.
  if (needSendKeyRelease) {
    needSendKeyRelease = false;
    blehid.keyRelease();
  }

  // if no new data, just return;
  if (!imuDataReady()) {
    return;
  }

  imuReadAndUpdateXYAngle();

  if (deviceMode == DEVICE_MOUSE_MODE) {
    processMouse();
  } else {
    processKeyboard();
  }
}

uint8_t clickButtons[] = {MOUSE_LEFT, MOUSE_RIGHT, MOUSE_ACTIVATE,
                          KEYPAD_ACTIVATE, DEVICE_SELECT};
uint8_t clickButtonLastState[] = {HIGH, HIGH, LOW, LOW, HIGH};
#ifdef PIMORONI_TRACKBALL
uint8_t clickButtonCode[] = {MOUSE_BUTTON_LEFT, MOUSE_BUTTON_LEFT, 0, 0, 0};
#else
uint8_t clickButtonCode[] = {MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT, 0, 0, 0};
#endif
uint8_t clickButtonKeyboardCode[] = {HID_KEY_ENTER, HID_KEY_BACKSPACE, 0, 0, 0};

void scanOneClickButton(uint8_t keyIndex) {

  // Do not switch mode or device in cases 1) mouse button pressed 2) keypressed
  if (noModeSwitch && (clickButtons[keyIndex] == MOUSE_ACTIVATE ||
                       clickButtons[keyIndex] == KEYPAD_ACTIVATE ||
                       clickButtons[keyIndex] == DEVICE_SELECT))
    return;

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
  clickButtonLastState[keyIndex] = state;

  switch (clickButtons[keyIndex]) {
  case MOUSE_ACTIVATE:
    // Serial.println("switch to mouse");

    deviceMode = DEVICE_MOUSE_MODE;

    // todo only reconfig when these is a real mode change. needed for bno085
    imuConfigure(deviceMode);
#ifdef PIMORONI_TRACKBALL
    trackball.setGreen(60);
    trackball.setBlue(0);
#endif
    break;
  case KEYPAD_ACTIVATE:
    // Serial.println("switch to keyboard");
    deviceMode = DEVICE_KEYBOARD_MODE;
    imuConfigure(deviceMode);
#ifdef PIMORONI_TRACKBALL
    trackball.setGreen(0);
    trackball.setBlue(80);
#endif
    break;
  case DEVICE_SELECT:
    if (state == LOW) {
      setDeviceId();
    }
    break;
  default:
    if (deviceMode == DEVICE_MOUSE_MODE) {
      if (state == LOW) {
        // if (keyIndex == 1) {
        //   // hack the backspace button for device switching
        //   setDeviceId();
        // } else {
        blehid.mouseButtonPress(clickButtonCode[keyIndex]);
        Serial.println("mouse button down");
        noModeSwitch = true;
        // }
      } else {
        blehid.mouseButtonRelease();
        Serial.println("mouse button up");
        noModeSwitch = false;
      }
    } else {
      if (state == LOW) {
        uint8_t keycodes[6] = {HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE,
                               HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE};
        // if (keyIndex == 1) {
        //   // hack the backspace button for device switching
        //   setDeviceId();
        // } else {
        keycodes[0] = clickButtonKeyboardCode[keyIndex];
        blehid.keyboardReport(0, keycodes);
        Serial.println("key button down");
        noModeSwitch = true;
        // }

      } else {
        blehid.keyRelease();
        Serial.println("key button up");
        noModeSwitch = false;
      }
    }
  }
}

void scanClickButtons() {

  // Only mouse left and right click
#ifdef PIMORONI_TRACKBALL
  for (int i = 1; i < 5; i++) {
    scanOneClickButton(i);
  }
#endif
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

  // Mystery of why !Serial not ready:
  // The "Serial" is always valid for an Arduino Uno, therefor that piece of
  // code does not wait. In the Leonardo, the "Serial" could be zero, if the
  // serial monitor has not been opened yet.

#ifdef IMU_USE_INT
  pinMode(IMU_INT, INPUT_PULLUP);
#endif

  pinMode(MOUSE_RIGHT, INPUT_PULLUP);
  pinMode(MOUSE_ACTIVATE, INPUT_PULLUP);
  pinMode(KEYPAD_ACTIVATE, INPUT_PULLUP);
  digitalWrite(KEYPAD_ACTIVATE, HIGH);
  digitalWrite(MOUSE_ACTIVATE, HIGH);
  digitalWrite(MOUSE_RIGHT, HIGH);

  pinMode(DEVICE_SELECT, INPUT_PULLUP);
  digitalWrite(DEVICE_SELECT, HIGH);

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
  Bluefruit.Advertising.clearData();
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
   * - Start(timeout) with timeout = 0 will advertise forever (until
   * connected)
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

  Bluefruit.begin();

  // Bluefruit.setAddr(&addr);
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16);
  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms

  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Rat0");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Ergo");
  bledis.setModel("Ergo");

  // Set PnP ID (includes VID, PID, and version)
  uint8_t pnp_id[7];
  pnp_id[0] = 0x01; // Vendor ID source: 0x01 = Bluetooth SIG, 0x02 = USB
                    // Implementer's Forum
  pnp_id[1] = (VENDOR_ID >> 8) & 0xFF;       // Vendor ID (high byte)
  pnp_id[2] = VENDOR_ID & 0xFF;              // Vendor ID (low byte)
  pnp_id[3] = (PRODUCT_ID >> 8) & 0xFF;      // Product ID (high byte)
  pnp_id[4] = PRODUCT_ID & 0xFF;             // Product ID (low byte)
  pnp_id[5] = (PRODUCT_VERSION >> 8) & 0xFF; // Product Version (high byte)
  pnp_id[6] = PRODUCT_VERSION & 0xFF;        // Product Version (low byte)
  bledis.setPNPID((const char *)pnp_id, 7);

  bledis.begin();
  blehid.begin();
  unsigned char addr[6];
  Bluefruit.getAddr(addr);
  addrByte3 = addr[3];

  // Set up and start advertising
  startAdv();
}

void printBDA() {
  ble_gap_addr_t addr = Bluefruit.getAddr();
  Serial.print("BDA type: ");
  Serial.print(addr.addr_type);
  for (int i = 0; i < 6; i++) {
    Serial.print(" ");
    Serial.print(addr.addr[i]);
  }
  Serial.println("");
}

void setBdDAAndName(unsigned char byte3, char *name) {
  /**
Set the local Bluetooth identity address.

The local Bluetooth identity address is the address that identifies this
device to other peers. The address type must be either @ref
BLE_GAP_ADDR_TYPE_PUBLIC or @ref BLE_GAP_ADDR_TYPE_RANDOM_STATIC.
Note
The identity address cannot be changed while advertising, scanning or creating a
connection. This address will be distributed to the peer during bonding. If the
address changes, the address stored in the peer device will not be valid and the
ability to reconnect using the old address will be lost. By default the
SoftDevice will set an address of type BLE_GAP_ADDR_TYPE_RANDOM_STATIC upon
being enabled. The address is a random number populated during the IC
manufacturing process and remains unchanged for the lifetime of each IC.
   *
   */
  Serial.print("before: ");
  printBDA();
  ble_gap_addr_t addr = Bluefruit.getAddr();

  Bluefruit.disconnect(Bluefruit.connHandle());
  Bluefruit.Advertising.stop();
  addr.addr[3] = byte3;
  Bluefruit.setAddr(&addr);
  startAdv();
  Bluefruit.setName(name);
  Serial.println(name);

  Serial.print("after: ");
  printBDA();
}

void setDeviceId() {
  if (deviceId == 0) {
    deviceId = 1;
    setBdDAAndName((unsigned char)(addrByte3 + 0x32), (char *)"Rat1");

#ifdef PIMORONI_TRACKBALL
    // flash white twice
    trackball.setWhite(240);
    delay(10);
    trackball.setWhite(0);
    delay(500);
    trackball.setWhite(240);
    delay(10);
    trackball.setWhite(0);
#endif
  } else {
    deviceId = 0;
    setBdDAAndName(addrByte3, (char *)"Rat0");
#ifdef PIMORONI_TRACKBALL
    // flash white once
    trackball.setWhite(240);
    delay(10);
    trackball.setWhite(0);
#endif
  }
}

void leds() {
  ledCount++;
  // pluse the green led to indicate system alive.
  if (ledCount % 10000 < 30) {
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

#ifdef PIMORONI_TRACKBALL

  // if (ledCount % 5000 == 0) {
  //   if (deviceId == 0) {
  //     trackball.setGreen(200);
  //   } else {
  //     trackball.setBlue(200);
  //   }
  // }
  // if (ledCount % 5000 == 100) {
  //   trackball.setGreen(0);
  //   trackball.setBlue(0);
  // }
#endif
}

#define MOUSE_JITTER 1
void processMouse() {
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

    y = (yAngle - lastYAngle) * SENSITIVITY_Y;

    // get rid of movement due to noise.
    if (abs(x) > MOUSE_JITTER || abs(y) > MOUSE_JITTER) {
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
        blehid.mouseMove(x, y);
      }
      lastXAngle = xAngle;
      lastYAngle = yAngle;
    }
  }
  // digitalWrite(DEBUG_3, LOW);
}
extern char buf[32][32];
void processKeyboard() {

  // Device in Keyboard mode

#ifdef IMU_LSM6DS3
  // Capture has not started, ignore until user activate keypad
  if (!startedChar) {
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
      return;
    } else {
      // User finger on the pad
      startedChar = true;
      imuStartSave(true);
    }
  }
#ifdef TSFLOW
  if (startedChar) {
    // User deactivated keypad
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
      imuStartSave(false);
      startedChar = false;
      if (imuPreprocessData()) {
        inference_started = true;
        imuDisplayPixelArray();

        tensorIndex = 0;
        for (int i = 0; i < 32; i++) {
          for (int j = 0; j < 32; j++) {
            tflInputTensor->data.f[tensorIndex++] = buf[i][j];
          }
        }
      };
    }
  }
#endif

  // return;
#endif

#ifdef TSFLOW
  if (inference_started) {
    inference_started = false;

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
    if (ch == '.') {
      digitalWrite(LED_RED, LIGHT_ON);
      delay(500);
      digitalWrite(LED_RED, LIGHT_OFF);
    }

    // Send KEY_DOWN
    if (ch != '.') {
      blehid.keyPress(ch);
      // Send KEY_UP at next loop
      needSendKeyRelease = true;
    }
  }
#endif
}