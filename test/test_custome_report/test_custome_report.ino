#include <bluefruit.h>
#include "local_constants.h"

BLEDis bledis;
BLEHidAdafruit blehid;

// Define your custom VID and PID
#define VENDOR_ID 0x3333        // Replace with your Vendor ID
#define PRODUCT_ID 0x5678       // Replace with your Product ID
#define PRODUCT_VERSION 0x0100  // Product version

void setup() {
  Serial.begin(115200);
  // while ( !Serial ) delay(10);

  Serial.println("Bluefruit52 HID Custom Report Example");
  Serial.println("--------------------------------------");

  Bluefruit.begin();
  Bluefruit.setName("Custom BLE HID");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Ergo");
  bledis.setModel("Ergo");

  // Set PnP ID (includes VID, PID, and version)
  uint8_t pnp_id[7];
  pnp_id[0] = 0x01;                           // Vendor ID source: 0x01 = Bluetooth SIG, 0x02 = USB Implementer's Forum
  pnp_id[1] = (VENDOR_ID >> 8) & 0xFF;        // Vendor ID (high byte)
  pnp_id[2] = VENDOR_ID & 0xFF;               // Vendor ID (low byte)
  pnp_id[3] = (PRODUCT_ID >> 8) & 0xFF;       // Product ID (high byte)
  pnp_id[4] = PRODUCT_ID & 0xFF;              // Product ID (low byte)
  pnp_id[5] = (PRODUCT_VERSION >> 8) & 0xFF;  // Product Version (high byte)
  pnp_id[6] = PRODUCT_VERSION & 0xFF;         // Product Version (low byte)
  bledis.setPNPID((const char*)pnp_id, 7);
  bledis.begin();

  // BLE HID
  blehid.begin();
  // Set the HID report map
  // blehid.setReportMap(desc_hid_report, sizeof(desc_hid_report));

  // Set up and start advertising
  startAdv();

  Serial.println("Advertising started");
}

void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

int count = 0;
void loop() {
  if (Bluefruit.connected()) {

    count++;
    if (count % 200000 == 0) {
      blehid.mouseMove(1, -1);
    }

    if (Serial.available() > 0)  // Check if data is available
    {
      char input = Serial.read();  // Read the next available byte

      // Echo the received data back to the serial monitor
      Serial.print("Received: ");
      Serial.println(input);

      // Process the input
      switch (input) {
        case 'w':                    // Up arrow key
          blehid.mouseMove(0, -10);  // Move mouse up
          break;
        case 's':                   // Down arrow key
          blehid.mouseMove(0, 10);  // Move mouse down
          break;
        case 'a':                    // Left arrow key
          blehid.mouseMove(-10, 0);  // Move mouse left
          break;
        case 'd':                   // Right arrow key
          blehid.mouseMove(10, 0);  // Move mouse right
          break;
        case 'z':
          blehid.consumerReport(0x3456);
          break;
        default:
          // Handle invalid input
          break;
      }
    }

    // uint8_t customReport[64];
    // for (int i = 0; i < 64; i++) {
    //   customReport[i] = i;  // Fill with incremental values as an example
    // }

    // // blehid.inputReport(1, customReport, sizeof(customReport));
    // blehid.consumerReport(0x3456);
    // blehid.mouseMove(1, 2);
    // // blehid.keySequence("test");

    // Serial.println("Sent custom report");
  }

  // delay(1000 * 10);
}