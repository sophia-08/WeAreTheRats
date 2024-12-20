#include "local_constants.h"

#include "BLEhidAdafruit1.h"
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

#include "lc3.h"
#include <vector>

int dt_us = 10000;
int sr_hz = 16000;

enum lc3_pcm_format pcm_format = LC3_PCM_FORMAT_S16;
void *lc3_encoder_mem = nullptr;
lc3_encoder_t lc3_encoder;

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
BLEHidAdafruit1 blehid;

int deviceId = 0;
unsigned addrByte3;

extern bool newData;
#define PDM_BUFFER_COUNT 8
uint8_t lc3Buffer[PDM_BUFFER_COUNT][LC3_OUTPUT_SIZE];
int lc3Index = 0;
int lc3SendIndex = 0;
int32_t mic;

bool pdmReady = false;
int pdmSkip;


int tp1 = 0, tp2 = 0;
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

void setup_encoder() {
  unsigned enc_size = lc3_encoder_size(dt_us, sr_hz);
  lc3_encoder_mem = malloc(enc_size);
  lc3_encoder = lc3_setup_encoder(dt_us, sr_hz, 0, lc3_encoder_mem);
}

void encode_one_frame(const int16_t *input_data) {

  // std::vector<uint8_t> output(LC3_OUTPUT_SIZE);
  // if (lc3Index < PDM_BUFFER_COUNT) {
    // lc3_encoder = lc3_setup_encoder(dt_us, sr_hz, 0, lc3_encoder_mem);
    lc3_encode(lc3_encoder, pcm_format, input_data, 1, LC3_OUTPUT_SIZE,
               &lc3Buffer[lc3Index % PDM_BUFFER_COUNT]);
    lc3Index++;
  // }
}

void setup() {
  deviceMode = DEVICE_MOUSE_MODE;
  configGpio();
  Serial.begin(115200);
  int i = 0;
  // while (!Serial) {
  //   digitalWrite(LED_RED, LIGHT_ON);
  //   delay(10);
  //   digitalWrite(LED_RED, LIGHT_OFF);
  //   delay(200);
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

  setup_encoder();

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
bool toSendEndOfStream = false;
bool toRequestConnInterval = false;

void loop() {
  toggleTp1();
  leds();
  scanClickButtons();

  // Request 7.5ms connection interval after connection
  // The host sends request too, so delay to let host request finish first
  if (toRequestConnInterval) {
    delay(2000);
    // get_conn_param(Bluefruit.connHandle());
    request_new_connection_interval(Bluefruit.connHandle());
    toRequestConnInterval = false;
  }

  if (lc3SendIndex < lc3Index - 1) {
#ifdef TEST_REPORT
    static uint8_t vv = 0;
    char vvBuffer[50];
    for (int f = 0; f < 50; f++) {
      vvBuffer[f] = f;
    }
    vv++;
    vvBuffer[1] = vv;
    blehid.consumerReport(vvBuffer, CUSTOMER_REPORT_SIZE);
#else
    blehid.consumerReport((char *)lc3Buffer[lc3SendIndex % PDM_BUFFER_COUNT],
                          CUSTOMER_REPORT_SIZE);
#endif
    toSendEndOfStream = true;

    Serial.print("S ");
    Serial.println(lc3SendIndex);
    lc3SendIndex += 2;
    toggleTp2();
  } else {
    if (toSendEndOfStream && deviceMode != DEVICE_VOICE_MODE) {
      char voiceoff[CUSTOMER_REPORT_SIZE];
      memset(voiceoff, 'f', CUSTOMER_REPORT_SIZE);
      voiceoff[0] = lc3Index & 0xff;
      voiceoff[1] = (lc3Index >> 8) & 0xff;
      blehid.consumerReport(voiceoff, CUSTOMER_REPORT_SIZE);
      toSendEndOfStream = false;
      toggleTp2();
      // toggleTp1() ;
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

  switch (deviceMode) {
  case DEVICE_MOUSE_MODE:
    processMouse();
    break;
  case DEVICE_KEYBOARD_MODE:
    processKeyboard();
    break;
  case DEVICE_VOICE_MODE:
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
uint8_t clickButtonKeyboardCode[] = {HID_KEY_ARROW_DOWN, HID_KEY_ARROW_UP, 0, 0, 0};


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

      lc3Index = 0;
      lc3SendIndex = 0;
      noModeSwitch = true;
      if (!PDM1.begin(1, 16000)) {
        Serial.println("Failed to start PDM!");
      } else {
        pdmReady = true;
        pdmSkip = 10;
      }
    } else {
      Serial.println("voice off");
      deviceMode = savedDeviceMode;
      noModeSwitch = false;
      pdmReady = false;
      PDM1.end();
      deviceMode = DEVICE_INACTIVE_MODE;
      sendVoiceDataToHost();
    }
    break;
  default:
  if (deviceMode == DEVICE_MOUSE_MODE) {
    if (state == LOW) {
      noModeSwitch = true;
      blehid.mouseButtonPress(clickButtonCode[keyIndex]);
      Serial.println("mouse button down");
    } else {
      blehid.mouseButtonRelease();
      Serial.println("mouse button up");
      noModeSwitch = false;
    }
  } else if (deviceMode == DEVICE_KEYBOARD_MODE) {
    if (state == LOW) {
      noModeSwitch = true;
      uint8_t keycodes[6] = {HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE,
                             HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE};
      keycodes[0] = clickButtonKeyboardCode[keyIndex];
      blehid.keyboardReport(0, keycodes);
      Serial.println("mouse button down");
    } else {        
      blehid.keyRelease();
      Serial.println("mouse button up");
      noModeSwitch = false;
    }
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

  Bluefruit.setEventCallback(on_ble_evt);
  Bluefruit.Periph.setConnectCallback(connect_callback);
}

void on_ble_evt(ble_evt_t *p_ble_evt) {
  if (p_ble_evt->header.evt_id == BLE_GAP_EVT_CONN_PARAM_UPDATE) {
    // Connection parameters updated event
    ble_gap_evt_conn_param_update_t *p_evt_conn_param_update =
        &p_ble_evt->evt.gap_evt.params.conn_param_update;

    float min_interval_ms =
        p_evt_conn_param_update->conn_params.min_conn_interval * 1.25;
    float max_interval_ms =
        p_evt_conn_param_update->conn_params.max_conn_interval * 1.25;

    Serial.print("Connection interval (min): ");
    Serial.print(min_interval_ms);
    Serial.println(" ms");

    Serial.print("Connection interval (max): ");
    Serial.print(max_interval_ms);
    Serial.println(" ms");

    // Actual connection interval is likely closer to max
    float actual_interval_ms =
        p_evt_conn_param_update->conn_params.max_conn_interval * 1.25;
    Serial.print("Actual connection interval: ");
    Serial.print(actual_interval_ms);
    Serial.println(" ms");
  }
}

// Callback when a connection is made
void connect_callback(uint16_t conn_handle) { toRequestConnInterval = true; }

void request_new_connection_interval(uint16_t conn_handle) {
  ble_gap_conn_params_t gap_conn_params;

  gap_conn_params.min_conn_interval = 6;
  gap_conn_params.max_conn_interval = 7;
  gap_conn_params.slave_latency = 0;
  gap_conn_params.conn_sup_timeout = 10;

  // Request the new connection parameters
  uint32_t err_code =
      sd_ble_gap_conn_param_update(conn_handle, &gap_conn_params);
  if (err_code == NRF_SUCCESS) {
    Serial.println("Connection interval update requested successfully.");
  } else {
    Serial.print("Error requesting connection interval update: ");
    Serial.println(err_code);
  }
}

void get_conn_param(uint16_t conn_handle) {
  ble_gap_conn_params_t gap_conn_params;
  uint32_t err_code = sd_ble_gap_ppcp_get(&gap_conn_params);
  if (err_code == NRF_SUCCESS) {
    Serial.print("Get conn param successfully. ");
    Serial.print(gap_conn_params.min_conn_interval);
    Serial.print(", ");
    Serial.println(gap_conn_params.max_conn_interval);
  } else {
    Serial.print("Error get conn param ");
    Serial.println(err_code);
  }
}

void initAndStartBLE() {

  Bluefruit.begin();

  // Bluefruit.setAddr(&addr);
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(6, 7);
  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms

  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Rat0");
  // Bluefruit.setMaxMtu(50);

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
return;
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

  // digitalWrite(TP2, HIGH);
  // query the number of bytes available
  int bytesAvailable = PDM1.available();
  // Serial.println(bytesAvailable);
  int16_t pdmBuffer[DEFAULT_PDM_BUFFER_SIZE/2];

  if (pdmSkip) {
    PDM1.read(pdmBuffer, bytesAvailable);
    pdmSkip--;
    return;
  }

  // toggleTp1();
  // read into the pdm buffer

  PDM1.read(pdmBuffer, bytesAvailable);

  encode_one_frame(pdmBuffer);
  // Serial.print("enc");
  // Serial.println(lc3Index);
  // for (int i = 0; i < 20; i++) {
  //   Serial.println(out[i]);
  // }
  // Serial.println(out.size());

  // 16-bit, 2 bytes per sample

  // digitalWrite(TP2, LOW);
}

void toggleTp1() {
  if (tp1) {
    tp1 = 0;
    digitalWrite(TP1, LOW);
  } else {
    tp1 = 1;
    digitalWrite(TP1, HIGH);
  }
}

void toggleTp2() {
  if (tp2) {
    tp2 = 0;
    digitalWrite(TP2, LOW);
  } else {
    tp2 = 1;
    digitalWrite(TP2, HIGH);
  }
}

void sendVoiceDataToHost() {
  int i;
  return;
  Serial.println("lc3");
  for (i = 0; i < lc3Index; i++) {
    for (int j = 0; j < LC3_OUTPUT_SIZE; j++) {
      Serial.println(lc3Buffer[i][j]);
    }
  }
  Serial.println("lc3_end");
}