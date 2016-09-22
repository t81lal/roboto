#include <Arduino.h>
#include "PSX.h"

Psx::Psx()
{
	rX = 0;
	rY = 0;
	lX = 0;
	lY = 0;
	mode = 0;
	digital_buttons = 0;
	smallMotor = 0;
	largeMotor = 0;
	_data1 = 0;
	_data2 = 0;

	_dataPin = 0;
	cmdPin = 0;
	_attPin = 0;
	_clockPin = 0;
	_delay = 0;
	i = 0;
	temp = 0;
	dataIn = 0;
}
#define CLKDELAY 20

#define WAIT 100
//byte Psx::shift(byte _dataOut) { return SPI.transfer(_dataOut);}

bool Psx::isDown(int button) {
	return (digital_buttons & button) == button;
}

byte Psx::shift(byte _dataOut)	// Does the actual shifting, both in and out simultaneously
{


	temp = 0;
	dataIn = 0;

	for (i = 0; i <= 7; i++)
	{
		digitalWrite(_clockPin, LOW);
		if ( _dataOut & (1 << i) ) digitalWrite(cmdPin, HIGH);	// Writes out the _dataOut bits
		else digitalWrite(cmdPin, LOW);
		temp = digitalRead(_dataPin);					// Reads the data pin
		if (temp)
		{
			dataIn = dataIn | (B00000001 << i);		// Shifts the read data into _dataIn
		}

		digitalWrite(_clockPin, HIGH);
	}
	return dataIn;
}

void Psx::setupPins(byte dataPin, byte cmndPin, byte attPin, byte clockPin)
{
	pinMode(dataPin, INPUT);
	digitalWrite(dataPin, HIGH);	// Turn on internal pull-up
	_dataPin = dataPin;

	pinMode(cmndPin, OUTPUT);
	cmdPin = cmndPin;

	pinMode(clockPin, OUTPUT);
	_clockPin = clockPin;
	digitalWrite(_clockPin, HIGH);

	pinMode(attPin, OUTPUT);
	_attPin = attPin;
	digitalWrite(_attPin, HIGH);


	smallMotor = 0x00;
	largeMotor = 0x00;
	//setup SPI
//  	SPI.mode((1<<SPR1) | (1<<SPR0) | (1<<DORD) | (1<<CPOL) | (1<<CPHA));
//  	SPSR = SPSR | (0<<SPI2X);

}

unsigned int Psx::poll()
{
	digitalWrite(_attPin, LOW);

	shift(0x01);
	mode = shift(0x42);
	shift(0x00);

	digital_buttons = (~shift(smallMotor) & 0x00FF); //motor (change value to FF to turn on motor in dualshock controller)
	digital_buttons |= (~shift(largeMotor) << 8); //motor (Large motor, will turn on for values over 40)
	if (0x70 == (mode & 0xF0))
	{
		rX = shift(0x00);
		rY = shift(0x00);
		lX = shift(0x00);
		lY = shift(0x00);
	}


	digitalWrite(_attPin, HIGH);

	return digital_buttons;
}

void Psx::Config_Enter()
{
	digitalWrite(_attPin, LOW); //goto configuration mode
	shift(0x01);
	shift(0x43);
	shift(0x00);
	shift(0x01);
	shift(0x00);
	digitalWrite(_attPin, HIGH);
}

void Psx::Config_Exit()
{
	digitalWrite(_attPin, LOW); //exit config mode
	shift(0x01);
	shift(0x43);
	shift(0x00);
	shift(0x00);
	shift(0x5A);
	shift(0x5A);
	shift(0x5A);
	shift(0x5A);
	shift(0x5A);
	digitalWrite(_attPin, HIGH);
}
//modes are 0x01 analog| 0x00 digital
void Psx::Config_Mode(byte mode)
{
	digitalWrite(_attPin, LOW);
	shift(0x01);
	shift(0x44);
	shift(0x00);
	shift(mode);
	shift(0x03);	//lock analog/digital mode select button on controller
	shift(0x00);
	shift(0x00);
	shift(0x00);
	shift(0x00);
	digitalWrite(_attPin, HIGH);
}

void Psx::Config_dualshock()
{
	digitalWrite(_attPin, LOW); //setup motor command mapping
	shift(0x01);
	shift(0x4D);
	shift(0x00);
	shift(0x00);
	shift(0x01);
	shift(0xFF);
	shift(0xFF);
	shift(0xFF);
	shift(0xFF);
	digitalWrite(_attPin, HIGH);
}

byte Psx::initcontroller(byte controller_mode)
{
	poll();
	Config_Enter();
	Config_Enter();
	Config_Mode(controller_mode);
	Config_Mode(controller_mode);
	Config_dualshock();
	Config_Exit();
	Config_Exit();
	Config_Exit();

	Config_Exit();
	poll();
	poll();
	poll();
	poll();
	return mode;
}
