/*
 * CalculationStructs.h
 *
 *  Created on: 19 Apr 2016
 *      Author: Bibl
 */

#ifndef CALCULATIONSTRUCTS_H_
#define CALCULATIONSTRUCTS_H_

#include "Constants.h"

struct BalanceState {
	long translationX;
	long translationZ;
	long translationY;
	long balanceY;
	long balanceX;
	long balanceZ;

	BalanceState() :
			translationX(0), translationY(0), translationZ(0),
			balanceX(0), balanceY(0), balanceZ(0)
	{
	}

	void zero() {
		translationX = 0;
		translationZ = 0;
		translationY = 0;
		balanceX = 0;
		balanceY = 0;
		balanceZ = 0;
	}

	void checkBalancingBounds() {
		if (balanceY > 0) {
			balanceY -= 1800;
		} else {
			balanceY += 1800;
		}
		if (balanceZ < -1800) {
			balanceZ += 3600;
		}
		if (balanceX < -1800) {
			balanceX += 3600;
		}
	}

	void scale(int factor) {
		translationZ = translationZ / factor;
		translationX = translationX / factor;
		translationY = translationY / factor;
	}

	void finalise(int factor) {
		balanceY = -balanceY / factor;
		balanceX = -balanceX / factor;
		balanceZ = balanceZ / factor;
	}
};

struct Vector3L {
	long x;
	long y;
	long z;

	Vector3L() : x(0), y(0), z(0) {
	}

	void zero() {
		x = 0;
		y = 0;
		z = 0;
	}

	void print() {
		Serial.print("vec, x=");
		Serial.print(x);
		Serial.print(", y=");
		Serial.print(y);
		Serial.print(", z=");
		Serial.println(z);
	}
};

typedef Vector3L MovementInput;

struct PositionTable {
	float xPos[6];
	float yPos[6];
	float zPos[6];
	short coxaAngles[6];
	short femurAngles[6];
	short tibiaAngles[6];

	void checkAngleBounds() {
		for (int i = 0; i <= 5; i++) {
			coxaAngles[i] = min(max(coxaAngles[i], (short)pgm_read_word(&MIN_COXA[i])), (short)pgm_read_word(&MAX_COXA[i]));
			femurAngles[i] = min(max(femurAngles[i], (short)pgm_read_word(&MIN_FEMUR[i])), (short)pgm_read_word(&MAX_FEMUR[i]));
			tibiaAngles[i] = min(max(tibiaAngles[i], (short)pgm_read_word(&MIN_TIBIA[i])), (short)pgm_read_word(&MAX_TIBIA[i]));
		}
	}
};

struct Gait {
	// gait - relative
	long gaitXPos[6];
	long gaitYPos[6];
	long gaitZPos[6];
	long gaitYRot[6];

	MovementInput selectedLegVector;

	// gait sequence data
	// init pos of legs
	byte legs[6];
	// lifted positions
	short liftPos;
	boolean shouldHalfList;
	// floor steps
	short stepFactor;
	byte length;
	short speed;

	byte selectedLeg;
	byte previousSelectedLeg;
	bool shouldHoldPositions;

	byte liftFactor;
	bool isLastLeg;
	short legLiftHeight;

	bool isInGait;
	byte index;
	byte gaitId;

	void setGait(int id) {
		// reset the gait position
		index = 1;
		gaitId = id;

		switch(id) {
			case 0:
				// 12 step ripple
				legs[LR] = 1;
				legs[RF] = 3;
				legs[LM] = 5;
				legs[RR] = 7;
				legs[LF] = 9;
				legs[RM] = 11;

				liftPos = 3;
				shouldHalfList = 3;
				stepFactor = 8;
				length = 12;
				speed = 70;
				break;
			case 1:
				// 8 step tripod
				legs[LR] = 5;
				legs[RF] = 1;
				legs[LM] = 1;
				legs[RR] = 1;
				legs[LF] = 5;
				legs[RM] = 5;

				liftPos = 3;
				shouldHalfList = 3;
				stepFactor = 4;
				length = 8;
				speed = 70;
				break;
			case 2:
				// 12 step triple tripod
				legs[RF] = 3;
				legs[LM] = 4;
				legs[RR] = 5;
				legs[LF] = 9;
				legs[RM] = 10;
				legs[LR] = 11;

				liftPos = 3;
				shouldHalfList = 3;
				stepFactor = 8;
				length = 12;
				speed = 60;
				break;
			case 3:
				// 12 step triple tripod using 5 lifted positions
				legs[RF] = 4;
				legs[LM] = 5;
				legs[RR] = 6;
				legs[LF] = 12;
				legs[RM] = 13;
				legs[LR] = 14;

				liftPos = 5;
				shouldHalfList = 1;
				stepFactor = 10;
				length = 16;
				speed = 60;
				break;
			case 4:
				// 24 step wave
				legs[LR] = 1;
				legs[RF] = 21;
				legs[LM] = 5;

				legs[RR] = 13;
				legs[LF] = 9;
				legs[RM] = 17;

				liftPos = 3;
				shouldHalfList = 3;
				stepFactor = 20;
				length = 24;
				speed = 70;
				break;
		}
	}

	void advanceGait(Vector3L &travelLength, int legIndex) {
		if (!isInGait) {
			travelLength.x = 0;
			travelLength.z = 0;
			travelLength.y = 0;
		}

		if ((isInGait && (liftPos == 1 || liftPos == 3 || liftPos == 5) && index == legs[legIndex])
				|| (!isInGait && index == legs[legIndex] && ((abs(gaitXPos[legIndex]) > 2) || (abs(gaitZPos[legIndex]) > 2) || (abs(gaitYRot[legIndex]) > 2)))) {
			// up
			gaitXPos[legIndex] = 0;
			gaitYPos[legIndex] = -legLiftHeight;
			gaitZPos[legIndex] = 0;
			gaitYRot[legIndex] = 0;
		} else if (((liftPos == 2 && index == legs[legIndex]) || (liftPos >= 3 && (index == legs[legIndex] - 1 || index == legs[legIndex] + (length - 1))))
				&& isInGait) {
			gaitXPos[legIndex] = -travelLength.x / liftFactor;
			gaitYPos[legIndex] = -3 * legLiftHeight / (3 + shouldHalfList); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
			gaitZPos[legIndex] = -travelLength.z / liftFactor;
			gaitYRot[legIndex] = -travelLength.y / liftFactor;
		} else if ((liftPos >= 2) && (index == legs[legIndex] + 1 || index == legs[legIndex] - (length - 1)) && isInGait) {
			gaitXPos[legIndex] = travelLength.x / liftFactor;
			gaitYPos[legIndex] = -3 * legLiftHeight / (3 + shouldHalfList); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
			gaitZPos[legIndex] = travelLength.z / liftFactor;
			gaitYRot[legIndex] = travelLength.y / liftFactor;
		} else if (((liftPos == 5 && (index == legs[legIndex] - 2))) && isInGait) {
			gaitXPos[legIndex] = -travelLength.x / 2;
			gaitYPos[legIndex] = -legLiftHeight / 2;
			gaitZPos[legIndex] = -travelLength.z / 2;
			gaitYRot[legIndex] = -travelLength.y / 2;
		} else if ((liftPos == 5) && (index == legs[legIndex] + 2 || index == legs[legIndex] - (length - 2)) && isInGait) {
			gaitXPos[legIndex] = travelLength.x / 2;
			gaitYPos[legIndex] = -legLiftHeight / 2;
			gaitZPos[legIndex] = travelLength.z / 2;
			gaitYRot[legIndex] = travelLength.y / 2;
		} else if ((index == legs[legIndex] + liftPos || index == legs[legIndex] - (length - liftPos)) && gaitYPos[legIndex] < 0) {
			// leg front down position
			gaitXPos[legIndex] = travelLength.x / 2;
			gaitZPos[legIndex] = travelLength.z / 2;
			gaitYRot[legIndex] = travelLength.y / 2;
			gaitYPos[legIndex] = 0;	// Only move leg down at once if terrain adaption is turned off
		} else {
			// move body forward
			gaitXPos[legIndex] = gaitXPos[legIndex] - (travelLength.x / stepFactor);
			gaitYPos[legIndex] = 0;
			gaitZPos[legIndex] = gaitZPos[legIndex] - (travelLength.z / stepFactor);
			gaitYRot[legIndex] = gaitYRot[legIndex] - (travelLength.y / stepFactor);
		}

		// next gait step
		if (isLastLeg) {
			index = index + 1;
			if (index > length) {
				index = 1;
			}
		}
	}

	void sequence(Vector3L &travelLength) {
		isInGait = ((abs(travelLength.x) > DEADZONE) || abs(travelLength.z) > DEADZONE) || (abs(travelLength.y) > DEADZONE);
		if (liftPos == 5) {
			liftFactor = 4;
		} else {
			liftFactor = 2;
		}

		isLastLeg = false;
		for (int i = 0; i <= 5; i++) {
			if (i == 5) {
				isLastLeg = true;
			}

			advanceGait(travelLength, i);
		}
	}

	int calculateMoveTime(Vector3L &travelLength, int speedControl, int inputDelay) {
		if ((abs(travelLength.x) > DEADZONE) || (abs(travelLength.z) > DEADZONE) || (abs(travelLength.y * 2) > DEADZONE)) {
			// extended time for balance mode
			return (speed + (inputDelay * 2) + speedControl) + 100;
		} else {
			// movement speed excluding walking
			return speedControl + 200;
		}
	}

	void alternateLegLiftHeight() {
		if (legLiftHeight == 50) {
			legLiftHeight = 80;
		} else {
			legLiftHeight = 50;
		}

		Serial.print("Set leg lift height to ");
		Serial.println(legLiftHeight);
	}
};

#endif /* CALCULATIONSTRUCTS_H_ */
