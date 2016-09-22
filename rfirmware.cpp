#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PSX.h"
#include "InverseKinematics.h"
#include "FastTrig.h"
#include "CalculationStructs.h"
#include "Constants.h"

#define STATE_COUNT 3
enum MovementState {ROTATE, TRANSLATE, WALK};

Psx controller;
FastTrig trig;
SoftwareSerial sscSerial(13, 12);
MovementInput positionInput;
MovementInput rotationInput;
Vector3L travelLength;
PositionTable table;
InverseKinematics kinematics(trig);
BalanceState balanceState;
Gait gait;
MovementState movementState;

word moveTime;
word previousMoveTime;
byte inputDelay;
int speedControl = 100;
bool isMoving;
bool shouldContinueWalking;

unsigned long timerStart;
unsigned long timerEnd;
byte cycleTime;

int yOffset;
int yShift;

bool shouldBalance;
bool debugMode;
bool shouldShowMenu;
bool isOnline;

void scheduleAngle(int angle, int legIndex, byte pinOffset) {
	sscSerial.print("#");
	sscSerial.print(pgm_read_byte(&PIN_LAYOUT[legIndex * 3]) + pinOffset, DEC);
	sscSerial.print("P");
	sscSerial.print(angle, DEC);
}
void scheduleMove(int legIndex, short coxaAngle, short femurAngle, short tibiaAngle) {
	byte sign = (legIndex <= 2) ? -1 : 1;
	int coxaOut = ((long) (sign * coxaAngle + 900)) * 1000 / PWM_DIV + PF_ADJUST;
	int femurOut = ((long) ((sign * femurAngle + 900)) * 1000 / PWM_DIV + PF_ADJUST);
	int tibiaOut = ((long) (sign * tibiaAngle + 900)) * 1000 / PWM_DIV + PF_ADJUST;

	scheduleAngle(coxaOut, legIndex, 0);
	scheduleAngle(femurOut, legIndex, 1);
	scheduleAngle(tibiaOut, legIndex, 2);
}
void commit() {
	sscSerial.print("T");
	sscSerial.println(moveTime, DEC);
}
void freeServos(void) {
	for (byte i = 0; i < 32; i++) {
		sscSerial.print("#");
		sscSerial.print(i, DEC);
		sscSerial.print("P0");
	}
	sscSerial.print("T200\r\n");
}

void setSpeedControlFactor(int val) {
	speedControl = max(0, min(2000, val));
	Serial.print("Set speed control factor to ");
	Serial.println(val);
}
void resetState() {
	freeServos();
	for (int i = 0; i <= 5; i++) {
		table.xPos[i] = (short) pgm_read_word(&INIT_POSX[i]);
		table.yPos[i] = (short) pgm_read_word(&INIT_POSY[i]);
		table.zPos[i] = (short) pgm_read_word(&INIT_POSZ[i]);
		table.coxaAngles[i] = 0;
		table.femurAngles[i] = 0;
		table.tibiaAngles[i] = 0;
	}
	positionInput.zero();
	rotationInput.zero();
	travelLength.zero();

	kinematics.setRotationOffsets(0, 0, 0);

	// selectedLeg = 255;
	// previousSelectedLeg = 255;

	gait.legLiftHeight = 50;
	gait.setGait(1);

	moveTime = 500;
	speedControl = 200;
	inputDelay = 128;

	yOffset = 65;
	yShift = 0;

	movementState = WALK;
	shouldBalance = true;
	isOnline = false;
	shouldShowMenu = true;
}

void setup() {
	Serial.begin(115200);
	sscSerial.begin(115200);

	Serial.println("startup");
	resetState();

	controller.setupPins(PS2_DAT, PS2_CMD, PS2_ATT, PS2_CLK);
	controller.initcontroller(0x1);
}

void handleInput() {
	if(controller.isDown(BUTTON_START)) {
		if(isOnline) {
			// turns it off
			resetState();
			delay(500);
		} else {
			isOnline = true;
			delay(500);
		}
		return;
	}

	if(!isOnline) {
		return;
	}

    if(controller.isDown(psxR2)) {
    	int nextState = (movementState + 1);
    	if(nextState >= STATE_COUNT) {
    		nextState = 0;
    	}
    	movementState = static_cast<MovementState>(nextState);
    	Serial.print("Switched to state ");
    	Serial.println(nextState);
    	delay(500);
    }

    if(controller.isDown(BUTTON_SQUARE)) {
    	shouldBalance = !shouldBalance;
    	Serial.print("Balancing mode is now ");
    	Serial.println(shouldBalance ? "enabled." : "disabled.");
    }

	if (controller.isDown(BUTTON_TRIANGLE)) {
		if (yOffset > 0) {
			yOffset = 0;
		} else {
			yOffset = 35;
		}
	}

	if (controller.isDown(BUTTON_UP)) {
		yOffset = max(min(yOffset + 5, MAX_BODY_Y_OFFSET), 0);
	}

	if (controller.isDown(BUTTON_DOWN)) {
		yOffset = max(yOffset - 5, 0);
	}

	if (controller.isDown(BUTTON_RIGHT)) {
		if (speedControl > 0) {
			setSpeedControlFactor(speedControl - 50);
			delay(250);
		}
	}

	if (controller.isDown(BUTTON_LEFT)) {
		if (speedControl < 2000) {
			setSpeedControlFactor(speedControl + 50);
			delay(250);
		}
	}

	yShift = 0;
	if (movementState == WALK) {
		if (controller.isDown(psxR1)) { // 2
			gait.alternateLegLiftHeight();
		}
	} else if (movementState == TRANSLATE) { // 1
		positionInput.x = (controller.lX - 128) / 2;
		positionInput.z = -(controller.lY - 128) / 3;
		positionInput.print();
		rotationInput.y = (controller.rX - 128) * 2;
		yShift = (-(controller.rY - 128) / 2);
	} else if(movementState == ROTATE) { // 0
		positionInput.x = (controller.lY - 128);
		positionInput.y = (controller.rX - 128) * 2;
		positionInput.z = (controller.lX - 128);
		yShift = (-(controller.rY - 128) / 2);
	}

    inputDelay = 128 - max(max(abs(controller.lX), abs(controller.lY)), abs(controller.rX));
	positionInput.y = max(yOffset + yShift, 0);
}

bool showMenu() {
	if(shouldShowMenu) {
		Serial.println("######====Command Monitor====######");
		Serial.println("      1. Change servo positions");
		Serial.println("      2. Debug");
		Serial.println("      3. Turn on.");
		Serial.println("      4. Reset SSC servo offets. (debug only)");
		Serial.println("      5. Show SSC servo offets.");
		Serial.println("      6. Show loaded servo offsets.");
		Serial.println("      7. Force reload loaded servo offsets.");
		Serial.println("      8. Free servos.");
		Serial.println("      9. Dump register contents.");
		shouldShowMenu = false;
	}

	byte buff[5];
	int read;

	delay(100);
	read = Serial.available();
	if (read > 0) {
		read = 0;
		for (read = 0; read < sizeof(buff); read++) {
			int ch = Serial.read();
			if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
				break;
			buff[read] = ch;
		}
		buff[read] = '\0';

		 if(read == 1) {
			shouldShowMenu = true;
			int opt = buff[0];

			switch(opt) {
				case 'd':
				case 'D':
					debugMode = !debugMode;
					Serial.print("Debug mode is now ");
					Serial.println((debugMode) ? "enabled" : "disabled.");
					break;
				case '1':
					// configureServos();
					break;
				case '2':
					// test();
					break;
				case '3':
					isOnline = true;
					break;
				case '4':
					if(debugMode) {
						// resetSSCOffsets();
					} else {
						Serial.println("Debug mode must be enabled to erase offsets.");
					}
					break;
				case '5':
					// printSSCOffsets();
					break;
				case '6':
					// printLoadedOffsets();
					break;
				case '7':
					// loadPWMOffsets();
					break;
				case '8':
					freeServos();
					break;
				case '9':
					// dumpRegisters();
					break;
			}
		}

		return true;
	}
	return false;
}

void loop() {
	controller.poll();
	handleInput();

	if(!isOnline) {
		showMenu();
		return;
	}

	timerStart = millis();

	gait.sequence(travelLength);
	if(shouldBalance) {
		kinematics.calculateBalanceTilts(&balanceState, gait, table);
	}
	int q = kinematics.calculatePositions(gait, table, balanceState, positionInput, rotationInput);
	if(q == SOLUTION_ERROR) {
		Serial.println("Solution error.");
	} else if(q == SOLUTION_WARN) {
		Serial.println("Solution warn.");
	}
	table.checkAngleBounds();

	moveTime = gait.calculateMoveTime(travelLength, speedControl, inputDelay);

	for(int i=0; i <= 5; i++) {
		scheduleMove(i, table.coxaAngles[i], table.femurAngles[i], table.tibiaAngles[i]);
	}

	shouldContinueWalking = false;

	// checking if gait pos/rot > or < 0
	for (int i = 0; i <= 5; i++) {
		if ((gait.gaitXPos[i] > 2) || (gait.gaitXPos[i] < -2)
				|| (gait.gaitYPos[i] > 2) || (gait.gaitYPos[i] < -2)
				|| (gait.gaitZPos[i] > 2) || (gait.gaitZPos[i] < -2)
				|| (gait.gaitYRot[i] > 2) || (gait.gaitYRot[i] < -2)) {
			shouldContinueWalking = true;
			break;
		}
	}

	if (isMoving || shouldContinueWalking) {
		word waitTime;
		isMoving = shouldContinueWalking;

		// endtime and waittime
		timerEnd = millis();
		if (timerEnd > timerStart) {
			cycleTime = timerEnd - timerStart;
		} else {
			cycleTime = 0xffffffffL - timerEnd + timerStart + 1;
		}
		// if it is less, use the last cycle time...
		// wait for previous commands to be completed while walking
		waitTime = (min(max ((previousMoveTime - cycleTime), 1), gait.speed));
		delay(waitTime);
	}

	commit();
	previousMoveTime = moveTime;
}
