/*
 * PSX.h
 *
 *  Created on: 24 Feb 2016
 *      Author: Bibl
 */

#ifndef PSX_H_
#define PSX_H_

//#include "WProgram.h"
#include "Arduino.h"
// Button Hex Representations:
//hat
#define BUTTON_LEFT		0x0080
#define BUTTON_DOWN		0x0040
#define BUTTON_RIGHT	0x0020
#define BUTTON_UP		0x0010
#define BUTTON_START		0x0008
#define BUTTON_SELECT		0x0001

//buttons
#define BUTTON_SQUARE		0x8000
#define BUTTON_CROSS		0x4000
#define BUTTON_CIRCLE		0x2000
#define BUTTON_TRIANGLE		0x1000

#define psxR1		0x0800
#define psxL1		0x0400
#define psxL2		0x0100
#define psxR2		0x0200
#define psxJoyL		0x0002
#define psxJoyR		0x0004
//other defines
#define psxAnalog	0x01
#define psxDigital	0x00



class Psx
{
	public:

		Psx();
		void setupPins(byte, byte, byte, byte);		// (Data Pin #, CMND Pin #, ATT Pin #, CLK Pin #, Delay)
									// Delay is how long the clock goes without changing state
									// in Microseconds. It can be lowered to increase response,
									// but if it is too low it may cause glitches and have some
									// keys spill over with false-positives. A regular PSX controller
									// works fine at 50 uSeconds.

		unsigned int poll();				// Returns the status of the button presses in an unsignd int.
									// The value returned corresponds to each key as defined above.

		byte initcontroller(byte); 		//initialize the controller (psxAnalog or psxDigital)
									//if controller is set to digital the right and left values in the Psx_Response
									//may be invalid
		bool isDown(int button);

		byte smallMotor;
		byte largeMotor;			//set the vibration motor feedback level for dualshock controllers
									//(Small motor, Large motor) Note the small motor will only turn on with
									//a value of 0xFF


		//note these are dangerous since these are unprotected, and they should be but this method this allows a faster program
		//because we dont need the overhead of function calls, (just be careful
		byte mode;
		unsigned int digital_buttons;
		byte rX;
		byte rY;
		byte lX;
		byte lY;

	private:
		void Config_dualshock();
		void Config_Mode(byte);
		void Config_Exit();
		void Config_Enter();

		byte shift(byte _dataOut);
		byte _dataPin;
		byte cmdPin;
		byte _attPin;
		byte _clockPin;

		byte _delay;
		byte i;
		boolean temp;
		byte dataIn;

		byte _data1;
		byte _data2;
};
#endif /* PSX_H_ */
