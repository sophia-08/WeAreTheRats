/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
// #include <Adafruit_LittleFS.h>
// #include <InternalFileSystem.h>

// BLE Service
BLEDis bledis;    // device information
BLEUart bleuart;  // uart over ble

#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D6
#define IMU_RESET D0
#define SWITCH_DEVICE_MODE D10
#define DEBUG_2 D2
#define DEBUG_3 D3
#define LIGHT_ON LOW
#define LIGHT_OFF HIGH
uint8_t buf[64];
void setup() {
  Serial.begin(115200);
  buf[0] = 'a';
  // #if CFG_DEBUG
  while (!Serial) delay(1);

  Serial.println("Tom cat\n");
  Serial.println("---------------------------\n");
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  // pinMode(LED_CHARGER, OUTPUT);
  pinMode(IMU_RESET, OUTPUT);
  pinMode(DEBUG_2, OUTPUT);
  pinMode(DEBUG_3, OUTPUT);
  //Reset IMU
  digitalWrite(IMU_RESET, HIGH);
  digitalWrite(IMU_RESET, LOW);
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
  // digitalWrite(LED_CHARGER, LIGHT_OFF);  // HIGH -- LED off.
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(9, 16);  // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  // To be consistent OTA DFU should be added first if it exists
  // bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Sophia");
  bledis.setModel("Tom");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  // blebas.begin();
  // blebas.write(100);

  // Set up and start advertising
  startAdv();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

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

int ledCount = 0;

void loop() {
  ledCount++;
  // HIGH  -- LIGHT_OFF
  if (ledCount % 50 == 0) {
    digitalWrite(LED_GREEN, LIGHT_ON);
  } else {
    digitalWrite(LED_GREEN, LIGHT_OFF);
  }

  if (digitalRead(MOUSE_ACTIVATE) == HIGH) {

    buf[0]++;
    if (buf[0] == 'z') {
      buf[0] = 'a';
    }
    bleuart.write(buf, 1);
    Serial.println((char)buf[0]);
    while (digitalRead(MOUSE_ACTIVATE) == HIGH)
      ;
  }

}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
}
