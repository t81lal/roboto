/*
 * InverseKinematics.h
 *
 *  Created on: 19 Apr 2016
 *      Author: Bibl
 */

#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <Arduino.h>
#include "FastTrig.h"
#include "CalculationStructs.h"

class InverseKinematics {
public:
	InverseKinematics(FastTrig &trig);
	virtual ~InverseKinematics();

	int calculatePositions(Gait &gait, PositionTable &table, BalanceState &bstate, MovementInput &positionInput, MovementInput &rotationInput);
	int calculateLegInverseKinematics(int legIndex, short feetPosX, short feetPosY, short feetPosZ, PositionTable &table);

	void calculateBalanceTilts(BalanceState *out,Gait &gait, PositionTable &table);
	void calculateLegBalance(BalanceState *out, int index, short posX, short posZ, short posY);
	void finaliseBalanceState(BalanceState *out);

	void calculateBodyMatrix(int legIndex, short posX, short posZ, short posY, short rotY, MovementInput &rotationInput, BalanceState &state, Vector3L &vector);
	void setRotationOffsets(short rx, short ry, short rz);
private:
	FastTrig &trig;
	short rotationOffsetX;
	short rotationOffsetY;
	short rotationOffsetZ;
};

#endif /* INVERSEKINEMATICS_H_ */
