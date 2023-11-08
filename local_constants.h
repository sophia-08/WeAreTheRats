#pragma once

#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D6
#define IMU_RESET D0
#define IMU_INT D0
#define KEYPAD_ACTIVATE D10
#define KEYPAD_CENTER D10
#define KEYPAD_UP D7
#define KEYPAD_RIGHT D3
#define KEYPAD_DOWN D2
#define KEYPAD_LEFT D1


// #define LED_CHARGER 23
#define LIGHT_ON LOW
#define LIGHT_OFF HIGH
#define BAT_CHARGE_STATE 23 // LOW for charging, HIGH not charging

// VBAT_ENABLE 14

// #define DEBUG_2 D2
// #define DEBUG_3 D3

#define SMOOTHING_RATIO 0.8
#define SENSITIVITY_X 30
#define SENSITIVITY_Y 35
#define DEBOUNCE_DELAY 5 // ms

#define DEVICE_MOUSE_MODE 0
#define DEVICE_KEYBOARD_MODE 1