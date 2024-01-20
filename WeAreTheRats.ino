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

#ifdef TSFLOW
#include "model.h"
#endif
#include "system.h"
void setBdDAAndName(unsigned char byte3, char *name);

// const float accelerationThreshold = 2.5; // threshold of significant in G's

int samplesRead = 0;
#define out_samples 150
int tensorIndex = 0;

int deviceMode;
BLEDis bledis;
BLEHidAdafruit blehid;

int deviceId = 0;
unsigned addrByte3;

#ifdef FEATURE_INERTIA_SCROLL
bool inertiaScroll = false;
#define INERTIA_SCROLL_DOWN 0
#define INERTIA_SCROLL_UP 1
bool inertiaScrollDirection = INERTIA_SCROLL_DOWN;
int inertiaScrollLastTimeStamp;
#endif

#ifdef TOM
// Central uart client
BLEClientUart clientUart;
#endif

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

// const uint8_t BLEUART_UUID_SERVICE[] =
// {
//     0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
//     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
// };
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
int xArrow=0, yArrow=0;

void loop() {

  leds();

#ifdef SEVEN_KEY_PAD
  scanNavigateButtons();
#endif

  scanClickButtons();

#ifdef FEATURE_INERTIA_SCROLL
  if (inertiaScroll) {
    if (millis() - inertiaScrollLastTimeStamp > 200) {
      inertiaScrollLastTimeStamp = millis();

      if (inertiaScrollDirection == INERTIA_SCROLL_DOWN) {
        blehid.mouseScroll(3);
      } else {
        blehid.mouseScroll(-3);
      }
      Serial.println("scroll");
    };
  }
#endif

  if (trackball.changed()) {
    // Bug of the hardware, int keep low.
    // https://github.com/pimoroni/pimoroni-pico/issues/357 if
    // (digitalRead(PIMORONI_TRACKBALL_INT) == LOW) {
    int x, y;
    if (deviceMode == DEVICE_MOUSE_MODE) {
      y = (trackball.right() - trackball.left()) * MINUTE_MOVEMENT;
      x = (-trackball.down() + trackball.up()) * MINUTE_MOVEMENT;
      if (x != 0 || y != 0) {
        Serial.print(x);
        Serial.print(",");
        Serial.println(y);
        blehid.mouseMove(x, y);
      }
      if (trackball.click()) {
        blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
        Serial.println("do");
      }
      if (trackball.release()) {

        blehid.mouseButtonRelease();

        Serial.println("re");
      }
    } else {
      y = (trackball.right() - trackball.left()) ;
      x = (-trackball.down() + trackball.up()) ;
      xArrow += x;
      yArrow += y;

      // keyboard mode
      uint8_t keycodes[6] = {HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE,
                             HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE};

      bool keyPressed = false;
      #define TRACKBALL_ARROW_KEY_SENSIVITY 5

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
    }
  }

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
                          KEYPAD_ACTIVATE};
uint8_t clickButtonLastState[] = {HIGH, HIGH, LOW, LOW};
uint8_t clickButtonCode[] = {MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT, 0, 0};
uint8_t clickButtonKeyboardCode[] = {HID_KEY_ENTER, HID_KEY_BACKSPACE, 0, 0};

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
  clickButtonLastState[keyIndex] = state;

  switch (clickButtons[keyIndex]) {
  case MOUSE_ACTIVATE:
    // Serial.println("switch to mouse");
    deviceMode = DEVICE_MOUSE_MODE;

    // todo only reconfig when these is a real mode change. needed for bno085
    imuConfigure(deviceMode);
    break;
  case KEYPAD_ACTIVATE:
    // Serial.println("switch to keyboard");
    deviceMode = DEVICE_KEYBOARD_MODE;
    imuConfigure(deviceMode);
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
        // }
      } else {
        blehid.mouseButtonRelease();
        Serial.println("mouse button up");
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
        // }

      } else {
        blehid.keyRelease();
        Serial.println("key button up");
      }
    }
  }
}

void scanClickButtons() {

  // Only mouse left and right click
  for (int i = 1; i < 4; i++) {
    scanOneClickButton(i);
  }
}

#ifdef SEVEN_KEY_PAD
#define DOUBLE_CLICK_INTERVAL 300
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

#ifdef FEATURE_INERTIA_SCROLL
    if (inertiaScroll) {
      inertiaScroll = false;
      Serial.println("end inertia scroll");
      return;
    }
#endif

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
#ifdef FEATURE_INERTIA_SCROLL
          inertiaScroll = true;
          inertiaScrollLastTimeStamp = millis();
          Serial.println("start inertia scroll");
          if (navigateButtons[keyIndex] == KEYPAD_LEFT) {
            inertiaScrollDirection = INERTIA_SCROLL_DOWN;
          } else {
            inertiaScrollDirection = INERTIA_SCROLL_UP;
          }

#else
          blehid.mouseButtonPress(navigateButtonDoubleClickMouseCode[keyIndex]);
          Serial.print("mouse db ");
          Serial.println(navigateButtonDoubleClickMouseCode[keyIndex]);
#endif
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

#endif

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

  // while (!Serial) {
  //   digitalWrite(LED_RED, LIGHT_ON);
  //   delay(10);
  //   digitalWrite(LED_RED, LIGHT_OFF);
  //   delay(100);
  // }

#ifdef IMU_USE_RESET
  pinMode(IMU_RESET, OUTPUT);
  // Reset IMU
  digitalWrite(IMU_RESET, HIGH);
  delay(0.1);
  digitalWrite(IMU_RESET, LOW);
  delay(0.1);
  digitalWrite(IMU_RESET, HIGH);
#endif

  pinMode(IMU_INT, INPUT_PULLUP);

  pinMode(MOUSE_RIGHT, INPUT_PULLUP);
  pinMode(MOUSE_ACTIVATE, INPUT_PULLUP);
  pinMode(KEYPAD_ACTIVATE, INPUT_PULLUP);
  digitalWrite(KEYPAD_ACTIVATE, HIGH);
  digitalWrite(MOUSE_ACTIVATE, HIGH);
  digitalWrite(MOUSE_RIGHT, HIGH);

#ifdef SEVEN_KEY_PAD
  pinMode(MOUSE_LEFT, INPUT_PULLUP);
  pinMode(KEYPAD_LEFT, INPUT_PULLUP);
  pinMode(KEYPAD_RIGHT, INPUT_PULLUP);
  pinMode(KEYPAD_CENTER, INPUT_PULLUP);
  pinMode(KEYPAD_UP, INPUT_PULLUP);
  pinMode(KEYPAD_DOWN, INPUT_PULLUP);

  digitalWrite(MOUSE_LEFT, HIGH);
  digitalWrite(KEYPAD_LEFT, HIGH);
  digitalWrite(KEYPAD_RIGHT, HIGH);
  digitalWrite(KEYPAD_CENTER, HIGH);
  digitalWrite(KEYPAD_UP, HIGH);
  digitalWrite(KEYPAD_DOWN, HIGH);
#endif

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

#ifdef TOM
  // Initialize Bluefruit with max concurrent connections as Peripheral = 1,
  // Central = 1. SRAM usage required by SoftDevice will increase with number
  // of connections
  Bluefruit.begin(1, 1);
  Bluefruit.Central.setConnInterval(100, 200);
  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(cent_connect_callback);
  Bluefruit.Central.setDisconnectCallback(cent_disconnect_callback);
#else
  Bluefruit.begin();
#endif
  // Bluefruit.setAddr(&addr);
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16);
  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms

  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Rat0");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Ergo");
  bledis.setModel("Ergo");
  bledis.begin();
  blehid.begin();
  unsigned char addr[6];
  Bluefruit.getAddr(addr);
  addrByte3 = addr[3];

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
  } else {
    deviceId = 0;
    setBdDAAndName(addrByte3, (char *)"Rat0");
  }
}

void leds() {
  ledCount++;
  // pluse the green led to indicate system alive.
  if (ledCount % 1000 < 30) {
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

  if (ledCount % 1000 == 0) {
    if (deviceMode == DEVICE_MOUSE_MODE) {
      trackball.setGreen(100);
    } else {
      trackball.setBlue(100);
    }
  }
  if (ledCount % 1000 == 100) {
    trackball.setGreen(0);
    trackball.setBlue(0);
  }
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

#ifdef BNO085
    // xAngle go back to 0 after pass 360 degrees. so here we need add the
    // offsets.
    if (x < -180 * SENSITIVITY_X) {
      x += 360 * SENSITIVITY_X;
    }
#endif

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
#ifdef BNO085
  // Capture has not started, ignore until user activate keypad
  if (!startedChar) {
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
      samplesRead = -1;
      return;
    } else {
      // User activate keypad, check whether 2s passed since last capture
      // int currentTime = millis();
      // if (currentTime < t1 + 2000) {
      //   return;
      // }
      // t1 = currentTime;
      startedChar = true;
      samplesRead = -1;
    }
  }

  // User finger is on keyboard_activation pad
  // To begin, wait 200ms
  // delay(200);

  // Loop to read 20 samples, at 100Hz, takes 200ms
  // This is better than delay, clear up data in IMU.
  for (int i = 0; i < 20;) {
    while (!imuDataReady()) {
    }
    imuReadNoWait();
    if (newData) {
      i++;
      newData = false;
    }
  }

  // Keep sampling until user release the ACTIVATE button
  while (true) {

    // User deactivated keypad
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
      startedChar = false;
      inference_started = true;
      break;
    }

    if (samplesRead >= out_samples) {
      // Wait for user release the button
      while (digitalRead(KEYPAD_ACTIVATE) == HIGH)
        ;
      startedChar = false;
      inference_started = true;
      break;
    }

    // BNO085 pull IMU_INT LOW when data is ready
    // so do nothing in case of IMU_INT high
    while (!imuDataReady()) {
    }
    imuReadNoWait();

    if (newData) {
      uint32_t now = micros();
      newData = false;

      // Wait for hand to rest
      if (samplesRead == -1) {
        if (imuSumOfAbsolateAcclOfAllAxis() > 1) {
          // Serial.println("wait idle");
          Serial.print("<");
          continue;
        }
        digitalWrite(LED_BLUE, LIGHT_ON);
        samplesRead = 0;
        continue;
      }

      // wait for hand to move
      if (samplesRead == 0) {
        if (imuSumOfAbsolateAcclOfAllAxis() < 2) {
          // Serial.println("wait move");
          Serial.print(">");
          continue;
        }
        tensorIndex = 0;
      }

      // Capture samples until keyboard_activation is release.
      imuSaveData(samplesRead);

      samplesRead++;
    }
  }

  // Not enough samples, restart
  if (samplesRead < 45) {
    Serial.print("not enough samples, ");
    Serial.println(samplesRead);
    samplesRead = -1;
    tensorIndex = 0;
    inference_started = false;
    startedChar = false;
    digitalWrite(LED_RED, LIGHT_ON);
    delay(500);
    digitalWrite(LED_RED, LIGHT_OFF);
    return;
  }

  Serial.print("tensor ");
  Serial.println(tensorIndex);

  // drop the last 5 points
  if (tensorIndex < out_samples * 9) {
    tensorIndex -= 5 * 9;
  }

  for (int i = tensorIndex; i < out_samples * 9; i++) {
    tflInputTensor->data.f[i] = 0;
  }
#endif

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