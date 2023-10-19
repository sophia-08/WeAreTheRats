#include <bluefruit.h>
#include "local_constants.h"

/**
 * @brief Use the given led indicate system halt. the led shall blink at
 * frequence of give duration
 *
 * @param led
 * @param seconds
 */
void systemHaltWithledPattern(int led, int seconds) {

  for (;;) {

    digitalWrite(led, LIGHT_ON);
    delay(1000 * seconds);

    digitalWrite(led, LIGHT_OFF);
    delay(1000 * seconds);
  }
}

void systemSleep() {
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[DEVICE_ACTIVATE],
                           NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  // power off
  sd_power_system_off();
  pinMode(DEVICE_ACTIVATE, INPUT_PULLUP);
}
