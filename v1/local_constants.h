#pragma once

#define PIMORONI_TRACKBALL
// #define SEVEN_KEY_PAD
// #define PIMORONI_TRACKBALL_INT D0
#define MINUTE_MOVEMENT 5

#ifdef PIMORONI_TRACKBALL
#define MOUSE_RIGHT D3
#define MOUSE_LEFT 255
#define DEVICE_SELECT D2
#endif

#ifdef SEVEN_KEY_PAD
#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define KEYPAD_CENTER D10
#define KEYPAD_UP D7
#define KEYPAD_RIGHT D3
#define KEYPAD_DOWN D2
#define KEYPAD_LEFT D1
#define DEVICE_SELECT 255
#endif

#define MOUSE_ACTIVATE D6
#define KEYPAD_ACTIVATE D10

// #define LED_CHARGER 23
#define LIGHT_ON LOW
#define LIGHT_OFF HIGH
#define BAT_CHARGE_STATE 23 // LOW for charging, HIGH not charging

// VBAT_ENABLE 14

// #define DEBUG_2 D2
// #define DEBUG_3 D3

#define SMOOTHING_RATIO 0.8
#define SENSITIVITY_X 45
#define SENSITIVITY_Y 50
#define DEBOUNCE_DELAY 5 // ms
#define TRACKBALL_ARROW_KEY_SENSIVITY 5

#define DEVICE_MOUSE_MODE 0
#define DEVICE_KEYBOARD_MODE 1

// #define BNO085
#ifdef BNO085
#define report_freq 1
#define BNO08X_RESET -1
#endif

#define IMU_LSM6DS3
#ifdef IMU_LSM6DS3
#define report_freq 10
#endif

// #define IMU_USE_RESET

// #define IMU_USE_INT
#ifdef IMU_USE_RESET
#define IMU_RESET D0
#endif

#ifdef IMU_USE_INT
#define IMU_INT D0
#endif
// #define TOM

#define TSFLOW
// #define FEATURE_INERTIA_SCROLL
