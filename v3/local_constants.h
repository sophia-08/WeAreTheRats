#pragma once

#define MINUTE_MOVEMENT 5

#define BUTTON1 D0
#define BUTTON2 D4
#define BUTTON3 D7

#define TP1 D5
#define TP2 D6

#define MOUSE_RIGHT BUTTON1
#define MOUSE_LEFT BUTTON3
// #define MOUSE_MIDDLE BUTTON2
#define DEVICE_SELECT BUTTON2
#define PDM_ACTIVATE BUTTON2

#define MOUSE_ACTIVATE D8
#define KEYPAD_ACTIVATE D1

// #define LED_CHARGER 23
#define LIGHT_ON LOW
#define LIGHT_OFF HIGH
#define BAT_CHARGE_STATE 23  // LOW for charging, HIGH not charging

// VBAT_ENABLE 14

// #define DEBUG_2 D2
// #define DEBUG_3 D3

#define SMOOTHING_RATIO 0.8
#define SENSITIVITY_X 45
#define SENSITIVITY_Y 50
#define DEBOUNCE_DELAY 5  // ms
#define TRACKBALL_ARROW_KEY_SENSIVITY 5

#define DEVICE_MOUSE_MODE 0
#define DEVICE_KEYBOARD_MODE 1
#define DEVICE_VOICE_MODE 2
#define DEVICE_INACTIVE_MODE 3

#define IMU_LSM6DS3
#ifdef IMU_LSM6DS3
#define report_freq 10
#endif

#ifdef IMU_USE_INT
#define IMU_INT D0
#endif

// #define TSFLOW