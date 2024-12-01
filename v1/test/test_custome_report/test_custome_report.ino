// #define CFG_DEBUG 1
#include "BLEhidAdafruit1.h"
#include "local_constants.h"
#include <bluefruit.h>

char banner_string[] = "clear; \
while true; do \
for eyes in \"-   -\" \"O   O\" \"O   O\" \"O   O\" \"o   o\" ; do \
clear; \
echo -e \"             .--,       .--,                                                \";\
echo -e \"            ( (  \\\\.---./  ) )                            ___                \";\
echo -e \"             '.__/\\e[0;31m${eyes}\\e[0m\\\\__.'                           .'o O'-._            \";\
echo -e \"                {=  \\e[0;31m^\\e[0m  =}                             / O o_.-\\`|            \";\
echo -e \"                 >  \\e[0;31m-\\e[0m  <                             /O_.-'  O |            \"; \
echo -e \"  ___________.\\\"\\\"\\`-------\\`\\\"\\\".____________            | o   o .-\\`         \";\
echo -e \" / Find me on github:                0  \\\\           |o O_.-'                \";\
echo -e \" \\\\  Ring:                      o        /           '--\\`                    \";\
echo -e \" /    sophia-08/WeAreTheRats            \\\\                                   \";\
echo -e \" \\\\  Companion App:                O     /         __                        \";\
echo -e \" /    sophia-08/HereComesTheChees       \\\\     _.-'  \\`.                      \";\
echo -e \" \\\\______________o__________o____________/ .-~^        \\`~--'                 \";\
echo -e \"               ___)( )(___        \\`-.___.'                                  \";\
echo -e \"             (((__)  (__)))                                                 \";\
sleep 0.5; \
done; \
done\n";


BLEDis bledis;
BLEHidAdafruit1 blehid;

// Define your custom VID and PID
#define VENDOR_ID 0x3333        // Replace with your Vendor ID
#define PRODUCT_ID 0x5678       // Replace with your Product ID
#define PRODUCT_VERSION 0x0100  // Product version

void setup() {
  Serial.begin(115200);
  digitalWrite(LED_BLUE, LIGHT_OFF);

  while (!Serial)
    delay(10);

  Serial.println("Bluefruit52 HID Custom Report Example");
  Serial.println("--------------------------------------");

  Bluefruit.begin();
  Bluefruit.setName("Custom BLE HID");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Ergo");
  bledis.setModel("Ergo");

  // Set PnP ID (includes VID, PID, and version)
  uint8_t pnp_id[7];
  pnp_id[0] = 0x01;                           // Vendor ID source: 0x01 = Bluetooth SIG, 0x02 = USB
                                              // Implementer's Forum
  pnp_id[1] = (VENDOR_ID >> 8) & 0xFF;        // Vendor ID (high byte)
  pnp_id[2] = VENDOR_ID & 0xFF;               // Vendor ID (low byte)
  pnp_id[3] = (PRODUCT_ID >> 8) & 0xFF;       // Product ID (high byte)
  pnp_id[4] = PRODUCT_ID & 0xFF;              // Product ID (low byte)
  pnp_id[5] = (PRODUCT_VERSION >> 8) & 0xFF;  // Product Version (high byte)
  pnp_id[6] = PRODUCT_VERSION & 0xFF;         // Product Version (low byte)
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
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
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
          {
            char buf[64];
            hid_keyboard_report_t report;
            varclr(&report);
            for (int i = 0; i < 64; i++) {
              buf[i] = i + 1;
            }
            blehid.consumerReport(buf, 20);

            delay(10 * 1000);
            blehid.keySequence(banner_string, 0.01);

            // report.keycode[0] = hid_ascii_to_keycode[(uint8_t)'c'][1];
            // report.keycode[1] = hid_ascii_to_keycode[(uint8_t)'l'][1];
            // report.keycode[2] = hid_ascii_to_keycode[(uint8_t)'e'][1];
            // report.keycode[3] = hid_ascii_to_keycode[(uint8_t)'a'][1];
            // report.keycode[4] = hid_ascii_to_keycode[(uint8_t)'r'][1];
            // report.keycode[5] = hid_ascii_to_keycode[(uint8_t)';'][1];
            // blehid.keyboardReport(&report);
            // blehid.keyRelease();
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
void set_keyboard_led(uint16_t conn_handle, uint8_t led_bitmap) {
  (void)conn_handle;

  Serial.print("Received led: ");
  Serial.println(led_bitmap);
}

void customer_cb(uint16_t conn_handle, char *data, uint16_t len) {
  (void)conn_handle;

  Serial.print("customer_cb: ");
  Serial.println(len);

  for (int i = 0; i < len; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }

  Serial.println(" ");
}
