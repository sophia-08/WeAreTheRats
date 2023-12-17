
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
