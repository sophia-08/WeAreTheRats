#include "local_constants.h"

#include "Adafruit_USBD_CDC.h"

#include "nrf_soc.h"

#include <bluefruit.h>

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
  //   nrf_gpio_cfg_sense_input(g_ADigitalPinMap[DEVICE_ACTIVATE],
  //                            NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  //   // power off
  //   sd_power_system_off();
  //   pinMode(DEVICE_ACTIVATE, INPUT_PULLUP);
  // nrf_gpio_cfg_sense_input(g_ADigitalPinMap[IMU_INT],
  //                          NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  // sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  // // nrf_pwr_mgmt_run();
  // attachInterrupt(IMU_INT, myinthandler, FALLING);  //RISING
  // sd_app_evt_wait();
  // pinMode(IMU_INT, INPUT_PULLUP);
}
