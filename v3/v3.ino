#include "local_constants.h"

#include "PDM1.h"
#include "battery.h"
#include "imu.h"
#include <bluefruit.h>
#include <microlzw.h>
#ifdef TSFLOW
#include "model.h"
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#endif

// Define your custom VID and PID
#define VENDOR_ID 0x3333       // Replace with your Vendor ID
#define PRODUCT_ID 0x5678      // Replace with your Product ID
#define PRODUCT_VERSION 0x0100 // Product version

#include "system.h"
void setBdDAAndName(unsigned char byte3, char *name);

int samplesRead = 0;
#define out_samples 150
int tensorIndex = 0;
bool noModeSwitch = false;

int deviceMode;
int savedDeviceMode;
BLEDis bledis;
BLEHidAdafruit blehid;

int deviceId = 0;
unsigned addrByte3;

extern bool newData;

// buffer to read samples into, each sample is 16-bits
#define PDM_BUFFER_SIZE (4 * 16 * 1000)
short pdmBuffer[PDM_BUFFER_SIZE];
int32_t mic;
int32_t pdmIndex = 0;
bool pdmReady = false;
int pdmSkip;
// number of samples read
volatile int pdmRead;

int tp1=0, tp2=0;
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
  while (!Serial) {
    digitalWrite(LED_RED, LIGHT_ON);
    delay(10);
    digitalWrite(LED_RED, LIGHT_OFF);
    delay(200);
    // if (++i > 1000) {
    //   break;
    // }
  }
  digitalWrite(LED_RED, LIGHT_OFF);

#ifdef TSFLOW
  loadTFLiteModel();
#endif

  initAndStartBLE();

  imuInit(deviceMode);

  // configure the data receive callback
  PDM1.onReceive(onPDMdata);
  // optionally set the gain, defaults to 20
  // PDM1.setGain(30);
  // initialize PDM1 with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  // if (!PDM1.begin(1, 16000)) {
  //   Serial.println("Failed to start PDM!");
  // }

  // calibrateIMU(250, 250);
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

  switch (deviceMode) {
  case DEVICE_MOUSE_MODE:
    processMouse();
    break;
  case DEVICE_KEYBOARD_MODE:
    processKeyboard();
    break;
  case DEVICE_VOICE_MODE:
    // pdmRead = 0;
    // if (pdmReady) {
    //   mic = getPDMwave(4000);
    //   Serial.print("Mic: ");
    //   Serial.println(mic);
    // }
    break;
  }
}

uint8_t clickButtons[] = {MOUSE_LEFT, MOUSE_RIGHT, MOUSE_ACTIVATE,
                          KEYPAD_ACTIVATE, DEVICE_SELECT};
uint8_t clickButtonLastState[] = {HIGH, HIGH, LOW, LOW, HIGH};

uint8_t clickButtonCode[] = {MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT, 0, 0, 0};

void scanOneClickButton(uint8_t keyIndex) {

  // Do not switch mode or device in cases 1) mouse button pressed 2) keypressed
  // 3)mouse movement is activated 4) keyboard is activated
  if (noModeSwitch && (clickButtons[keyIndex] == MOUSE_ACTIVATE ||
                       clickButtons[keyIndex] == KEYPAD_ACTIVATE
                       // ||clickButtons[keyIndex] == DEVICE_SELECT
                       ))
    return;

  uint8_t state = digitalRead(clickButtons[keyIndex]);
  if (state == clickButtonLastState[keyIndex]) { // no change
    return;
  }

  delay(1);
  state = digitalRead(clickButtons[keyIndex]);
  if (state == clickButtonLastState[keyIndex]) { // no change
    return;
  }

  //  edge is detected
  clickButtonLastState[keyIndex] = state;

  switch (clickButtons[keyIndex]) {
  case MOUSE_ACTIVATE:
    if (state == HIGH) {
      // noModeSwitch = true;
      Serial.println("mouse on");

      deviceMode = DEVICE_MOUSE_MODE;
      // todo only reconfig when these is a real mode change. needed for bno085
      imuConfigure(deviceMode);
    } else {
      // noModeSwitch = false;
      Serial.println("mouse off");
    }

    break;
  case KEYPAD_ACTIVATE:
    if (state == HIGH) {
      // noModeSwitch = true;
      Serial.println("keyboard on");
      deviceMode = DEVICE_KEYBOARD_MODE;
      imuConfigure(deviceMode);
    } else {
      // noModeSwitch = false;
      Serial.println("keyboard off");
    }
    break;
  case DEVICE_SELECT:
    // if (state == LOW) {
    //   setDeviceId();
    // }
    if (state == LOW) {
      Serial.println("voice on");
      savedDeviceMode = deviceMode;
      deviceMode = DEVICE_VOICE_MODE;
      pdmIndex = 0;
      noModeSwitch = true;
      if (!PDM1.begin(1, 16000)) {
        Serial.println("Failed to start PDM!");
      } else {
        pdmReady = true;
        pdmSkip = 10;
      }
    } else {
      Serial.print("voice off ");
      Serial.println(pdmIndex);
      deviceMode = savedDeviceMode;
      noModeSwitch = false;
      pdmReady = false;
      PDM1.end();
      sendVoiceDataToHost();
    }
    break;
  default:
    if (state == LOW) {
      noModeSwitch = true;
      blehid.mouseButtonPress(clickButtonCode[keyIndex]);
      Serial.println("mouse button down");
    } else {
      blehid.mouseButtonRelease();
      Serial.println("mouse button up");
      noModeSwitch = false;
    }
  }
}

void scanClickButtons() {
  for (int i = 0; i < 5; i++) {
    scanOneClickButton(i);
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

  pinMode(TP1, OUTPUT);
  pinMode(TP2, OUTPUT);
  // Mystery of why !Serial not ready:
  // The "Serial" is always valid for an Arduino Uno, therefor that piece of
  // code does not wait. In the Leonardo, the "Serial" could be zero, if the
  // serial monitor has not been opened yet.

#ifdef IMU_USE_INT
  pinMode(IMU_INT, INPUT_PULLUP);
#endif

  pinMode(MOUSE_LEFT, INPUT_PULLUP);
  pinMode(MOUSE_RIGHT, INPUT_PULLUP);
  pinMode(MOUSE_ACTIVATE, INPUT_PULLDOWN);
  pinMode(KEYPAD_ACTIVATE, INPUT_PULLDOWN);
  digitalWrite(KEYPAD_ACTIVATE, HIGH);
  digitalWrite(MOUSE_ACTIVATE, HIGH);
  digitalWrite(MOUSE_RIGHT, HIGH);
  digitalWrite(MOUSE_LEFT, HIGH);

  pinMode(DEVICE_SELECT, INPUT_PULLUP);
  digitalWrite(DEVICE_SELECT, HIGH);

  digitalWrite(LED_RED, LIGHT_OFF);
  digitalWrite(LED_BLUE, LIGHT_OFF);
  digitalWrite(LED_GREEN, LIGHT_OFF);

  digitalWrite(TP1, LOW);
  digitalWrite(TP2, LOW);
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

  } else {
    deviceId = 0;
    setBdDAAndName(addrByte3, (char *)"Rat0");
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

#ifdef TSFLOW
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

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM1.available();

  if (pdmSkip) {
    PDM1.read(&pdmBuffer[pdmIndex], bytesAvailable);
    pdmSkip--;
    return;
  }

   toggleTp1();
  // read into the pdm buffer
  if (pdmIndex + bytesAvailable / 2 >= PDM_BUFFER_SIZE) {
    pdmIndex -= bytesAvailable / 2;
  }
  PDM1.read(&pdmBuffer[pdmIndex], bytesAvailable);

  // 16-bit, 2 bytes per sample
  pdmRead = bytesAvailable / 2;
  pdmIndex += pdmRead;

  // Serial.println(pdmIndex);
}

void toggleTp1() {
  if (tp1) {
    tp1 = 0;
    digitalWrite(TP1, LOW);
  }else{
    tp1 = 1;
    digitalWrite(TP1, HIGH);
  }
}

void sendVoiceDataToHost() {
  int i;
  Serial.println("rec_ok");
  for (i = 0; i < pdmIndex; i++) {
    Serial.println(pdmBuffer[i]);
  }
  Serial.println("fi");
}