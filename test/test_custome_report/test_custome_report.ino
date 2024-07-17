// #define CFG_DEBUG 1
#include "local_constants.h"
#include <bluefruit.h>
#include "BLEhidAdafruit1.h"

BLEDis bledis;
BLEHidAdafruit1 blehid;

// Define your custom VID and PID
#define VENDOR_ID 0x3333       // Replace with your Vendor ID
#define PRODUCT_ID 0x5678      // Replace with your Product ID
#define PRODUCT_VERSION 0x0100 // Product version

void setup() {
  Serial.begin(115200);
  digitalWrite(LED_BLUE, LIGHT_OFF);

  while ( !Serial ) delay(10);

  Serial.println("Bluefruit52 HID Custom Report Example");
  Serial.println("--------------------------------------");

  Bluefruit.begin();
  Bluefruit.setName("Custom BLE HID");

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

  // BLE HID
  blehid.begin();
  // Set the HID report map
  // blehid.setReportMap(desc_hid_report, sizeof(desc_hid_report));
  // Set callback for set LED from central
  blehid.setKeyboardLedCallback(set_keyboard_led);
  blehid.setCustomerCallback(customer_cb);

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
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0); // 0 = Don't stop advertising after n seconds
}

int count = 0;
void loop() {
  count++;
  if (count % 10000 < 30) {
    digitalWrite(LED_GREEN, LIGHT_ON);
  } else {
    digitalWrite(LED_GREEN, LIGHT_OFF);
  }

  if (Bluefruit.connected()) {

    if (count % 200000 == 0) {
      blehid.mouseMove(1, -1);
    }

    if (Serial.available() > 0) // Check if data is available
    {
      char input = Serial.read(); // Read the next available byte

      // Echo the received data back to the serial monitor
      Serial.print("Received: ");
      Serial.println(input);

      // Process the input
      switch (input) {
      case 'w':                   // Up arrow key
        blehid.mouseMove(0, -10); // Move mouse up
        break;
      case 's':                  // Down arrow key
        blehid.mouseMove(0, 10); // Move mouse down
        break;
      case 'a':                   // Left arrow key
        blehid.mouseMove(-10, 0); // Move mouse left
        break;
      case 'd':                  // Right arrow key
        blehid.mouseMove(10, 0); // Move mouse right
        break;
      case 'z': {
        char buf[64];
        for (int i=0; i<64; i++) {
          buf[i] = i+1;
        }
        blehid.consumerReport(buf, 20);        
      }

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

/**
 * Callback invoked when received Set LED from central.
 * Must be set previously with setKeyboardLedCallback()
 *
 * The LED bit map is as follows: (also defined by KEYBOARD_LED_* )
 *    Kana (4) | Compose (3) | ScrollLock (2) | CapsLock (1) | Numlock (0)
 */
void set_keyboard_led(uint16_t conn_handle, uint8_t led_bitmap)
{
  (void) conn_handle;
  
Serial.print("Received led: ");
Serial.println(led_bitmap);
}

void customer_cb(uint16_t conn_handle, uint8_t led_bitmap)
{
  (void) conn_handle;
  
Serial.print("customer_cb: ");
Serial.println(led_bitmap);
}
