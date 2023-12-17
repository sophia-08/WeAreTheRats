#include "local_constants.h"
#include <bluefruit.h>
#define VBAT_PER_LBS (0.003515625F) // 3.6 reference and 10 bit resolution

float GetBatteryVoltage() {
  // digitalWrite(VBAT_ENABLE, LOW);

  uint32_t adcCount = analogRead(PIN_VBAT);
  float adcVoltage = adcCount * VBAT_PER_LBS;
  float vBat = adcVoltage * (1510.0 / 510.0);

  // digitalWrite(VBAT_ENABLE, HIGH);

  return vBat;
}

bool IsChargingBattery() { return digitalRead(BAT_CHARGE_STATE) == LOW; }
