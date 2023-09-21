
#include <Arduino.h>
#include "bluefruit_common.h"
// the setup function runs once when you press reset or power the board
void setup() {
// the pins are defined in bsp package **/packages/**vendor/**hardware/nrf52/*version/variants/boardname file
// variant.h define the index, variant.cpp has the array.

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);   
  pinMode(23, OUTPUT);   
   

  pinMode(D7, INPUT_PULLUP);
  pinMode(D8, INPUT_PULLUP);
  pinMode(D9, INPUT_PULLUP);
  pinMode(D10, INPUT_PULLUP);
  
  digitalWrite(D7, HIGH);
  digitalWrite(D8, HIGH);
  digitalWrite(D9, HIGH);
  digitalWrite(D10, HIGH);
}


void loop() {
  // The  leds shall light up after push keys
  digitalWrite(LED_BLUE, digitalRead(D7));  
  digitalWrite(LED_RED, digitalRead(D8));  
  digitalWrite(LED_GREEN, digitalRead(D9));  
  digitalWrite(23, digitalRead(D10));  
}
