/*
 *	Arduino library for the Pimoroni RGBW trackball breakout
 *
 *	https://github.com/ncmreynolds/pimoroniTrackball
 *
 *	Released under LGPL-2.1 see https://github.com/ncmreynolds/pimoroniTrackball/LICENSE for full license
 *
 *	This library implements similar functionality to https://github.com/pimoroni/trackball-python/ but in C++ for use in the Arduino IDE
 *
 *
 */
#ifndef pimoroniTrackball_h
#define pimoroniTrackball_h
#include <Arduino.h>
#include <Wire.h>

#define TRACKBALL_REG_LED_RED 0x00
#define TRACKBALL_REG_LED_GRN 0x01
#define TRACKBALL_REG_LED_BLU 0x02
#define TRACKBALL_REG_LED_WHT 0x03

#define TRACKBALL_REG_LEFT 0x04
#define TRACKBALL_REG_RIGHT 0x05
#define TRACKBALL_REG_UP 0x06
#define TRACKBALL_REG_DOWN 0x07
#define TRACKBALL_REG_SWITCH 0x08
#define TRACKBALL_MSK_SWITCH_STATE 0b10000000

#define TRACKBALL_REG_USER_FLASH 0xD0
#define TRACKBALL_REG_FLASH_PAGE 0xF0
#define TRACKBALL_REG_INT 0xF9
#define TRACKBALL_MSK_INT_TRIGGERED 0b00000001
#define TRACKBALL_MSK_INT_OUT_EN 0b00000010
#define TRACKBALL_REG_CHIP_ID_L 0xFA
#define TRACKBALL_RED_CHIP_ID_H 0xFB
#define TRACKBALL_REG_VERSION 0xFC
#define TRACKBALL_REG_I2C_ADDR 0xFD
#define TRACKBALL_REG_CTRL 0xFE
#define TRACKBALL_MSK_CTRL_SLEEP 0b00000001
#define TRACKBALL_MSK_CTRL_RESET 0b00000010
#define TRACKBALL_MSK_CTRL_FREAD 0b00000100
#define TRACKBALL_MSK_CTRL_FWRITE 0b00001000

class pimoroniTrackball
{

	public:
		pimoroniTrackball();													//Constructor function
		~pimoroniTrackball();													//Destructor function
		//Setup functions
		void begin(uint8_t address = 0x0A, TwoWire &i2cPort = Wire);			//Initialise the trackball library
		bool isConnected();														//Check the trackball is connected
		void setRGBW(uint8_t redBrightness, uint8_t greenBrightness, uint8_t blueBrightness, uint8_t whiteBrightness);	//Set the brightness for each of the RGB&W LEDs
		void setRed(uint8_t redBrightness);										//Set the brightness of the red LED
		void setGreen(uint8_t greenBrightness);									//Set the brightness of the green LED
		void setBlue(uint8_t blueBrightness);									//Set the brightness of the blue LED
		void setWhite(uint8_t whiteBrightness);									//Set the brightness of the white LED
		bool changed();															//Poll the trackball to see if it's moved
		uint8_t left();															//Amount the trackball moved left
		uint8_t right();														//Amount the trackball moved right
		uint8_t up();															//Amount the trackball moved up
		uint8_t down();															//Amount the trackball moved down
		bool click();															//Was the trackball clicked
		bool release();															//Was the trackball released
    void enable_interrupt();
	protected:
	private:
		int _trackballAddress = 0x0A;										//I2C address for the trackball
		TwoWire *_i2cPort = nullptr;										//Pointer to I2C port used by library
		const uint16_t trackballDeviceId = 0xBA11;							//Device ID for the trackball
		uint8_t _lastState[5] = {0, 0, 0, 0, 0b10000000};					//Last state of the trackball, which resets on read
};

extern pimoroniTrackball trackball;
#endif
