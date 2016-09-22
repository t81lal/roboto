/*
 * InverseKinematics.cpp
 *
 *  Created on: 19 Apr 2016
 *      Author: Bibl
 */

#include "InverseKinematics.h"
#include "Constants.h"

const float BALANCE_FACTOR = 6;

InverseKinematics::InverseKinematics(FastTrig &_trig) : trig(_trig),
rotationOffsetX(0), rotationOffsetY(0), rotationOffsetZ(0)
{
}

InverseKinematics::~InverseKinematics() {
}

int InverseKinematics::calculatePositions(Gait &gait, PositionTable &table, BalanceState &bstate, MovementInput &positionInput, MovementInput &rotationInput) {
	int quality = SOLUTION_FOUND;

	Vector3L globalRotationMatrix = Vector3L();
	for(int i=0; i <= 2; i++) {
		short x = -table.xPos[i] + positionInput.x + gait.gaitXPos[i] - bstate.translationX;
		short z = table.zPos[i] + positionInput.z + gait.gaitZPos[i] - bstate.translationZ;
		short y = table.yPos[i] + positionInput.y + gait.gaitYPos[i] - bstate.translationY;
		calculateBodyMatrix(i, x, z, y, (short)gait.gaitYRot[i], rotationInput, bstate, globalRotationMatrix);

		x = table.xPos[i] - positionInput.x + globalRotationMatrix.x - (gait.gaitXPos[i] - bstate.translationX);
		y = table.yPos[i] + positionInput.y - globalRotationMatrix.y + gait.gaitYPos[i] - bstate.translationY;
		z = table.zPos[i] + positionInput.z - globalRotationMatrix.z + gait.gaitZPos[i] - bstate.translationZ;
		int q = calculateLegInverseKinematics(i, x, y, z, table);
		if(q > quality) {
			quality = q;
		}
	}

	for(int i=3; i <= 5; i++) {
		short x = table.xPos[i] - positionInput.x + gait.gaitXPos[i] - bstate.translationX;
		short z = table.zPos[i] + positionInput.z + gait.gaitZPos[i] - bstate.translationZ;
		short y = table.yPos[i] + positionInput.y + gait.gaitYPos[i] - bstate.translationY;
		calculateBodyMatrix(i, x, z, y, (short)gait.gaitYRot[i], rotationInput, bstate, globalRotationMatrix);

		x = table.xPos[i] + positionInput.x - globalRotationMatrix.x + gait.gaitXPos[i] - bstate.translationX;
		y = table.yPos[i] + positionInput.y - globalRotationMatrix.y + gait.gaitYPos[i] - bstate.translationY;
		z = table.zPos[i] + positionInput.z - globalRotationMatrix.z + gait.gaitZPos[i] - bstate.translationZ;
		int q = calculateLegInverseKinematics(i, x, y, z, table);
		if(q > quality) {
			quality = q;
		}
	}

	return quality;
}

void InverseKinematics::calculateBodyMatrix(int legIndex, short posX, short posZ, short posY, short rotY, MovementInput &rotationInput, BalanceState &state, Vector3L &vector) {
	// centre to feet
	short xOffset = (short) pgm_read_word(&OFFSET_COXA_X[legIndex]) + posX + rotationOffsetX;
	short yOffset = posY + rotationOffsetY; // rotate around the y axis
	short zOffset = (short) pgm_read_word(&OFFSET_COXA_Z[legIndex]) + posZ + rotationOffsetZ;

	AngleTuple alpha = trig.sincos(rotationInput.x + state.balanceX);                 // x
	AngleTuple beta = trig.sincos(rotationInput.z + state.balanceZ);                  // z
	AngleTuple theta = trig.sincos(rotationInput.y + (rotY * POW1) + state.balanceY); // y

	// successive global rotation matrix
	vector.x = ((long) xOffset * POW2 - ((long) xOffset * POW2 * alpha.cos / POW4 * beta.cos / POW4 - (long) zOffset * POW2 * beta.cos / POW4 * alpha.sin / POW4 + (long) yOffset * POW2 * beta.sin / POW4)) / POW2;
	vector.z = ((long) zOffset * POW2 - ((long) xOffset * POW2 * theta.cos / POW4 * alpha.sin / POW4 + (long) xOffset * POW2 * alpha.cos / POW4 * beta.sin / POW4 * theta.sin / POW4 + (long) zOffset * POW2 * alpha.cos / POW4 * theta.cos / POW4 - (long) zOffset * POW2 * alpha.sin / POW4 * beta.sin / POW4 * theta.sin / POW4 - (long) yOffset * POW2 * beta.cos / POW4 * theta.sin / POW4)) / POW2;
	vector.y = ((long) yOffset * POW2 - ((long) xOffset * POW2 * alpha.sin / POW4 * theta.sin / POW4 - (long) xOffset * POW2 * alpha.cos / POW4 * theta.cos / POW4 * beta.sin / POW4 + (long) zOffset * POW2 * alpha.cos / POW4 * theta.sin / POW4 + (long) zOffset * POW2 * theta.cos / POW4 * alpha.sin / POW4 * beta.sin / POW4 + (long) yOffset * POW2 * beta.cos / POW4 * theta.cos / POW4)) / POW2;
}

int InverseKinematics::calculateLegInverseKinematics(int legIndex, short feetPosX, short feetPosY, short feetPosZ, PositionTable &table) {
	unsigned long legLength; // shoulder to wrist (SW) length
	unsigned long legAngle; // angle of SW to ground in rads
	unsigned long femurAngle; // angle of SW to femur in rads

	// calculate coxa angle
	ATan2Tuple atan2 = trig.atan2(feetPosX, feetPosZ);
	table.coxaAngles[legIndex] = (((long) atan2.angle * 180) / 3141) + (short) pgm_read_word(&COXA_ANGLES[legIndex]);

	short feetXZ = atan2.hyp / POW2; // length between coxa and tars

	// solve a1 and sw using atan2
	atan2 = trig.atan2(feetPosY, feetXZ - (byte) pgm_read_word(&COXA_LENGTHS[legIndex]));
	legAngle = atan2.angle; // a14 - angle between SW line and the ground in rads
	legLength = atan2.hyp;   // sw2 - length between tars and femur axis

	// a2 - angle of SW to femur in rads
	long b1N = ((((long) (byte) pgm_read_word(&FEMUR_LENGTHS[legIndex]) * (byte) pgm_read_word(&FEMUR_LENGTHS[legIndex])) - ((long) (byte) pgm_read_word(&TIBIA_LENGTHS[legIndex]) * (byte) pgm_read_word(&TIBIA_LENGTHS[legIndex]))) * POW4 + ((long) legLength * legLength));
	long b1D = (long) (2 * (byte) pgm_read_word(&FEMUR_LENGTHS[legIndex])) * POW2 * (unsigned long) legLength;
	long b1 = b1N / (b1D / POW4);
	femurAngle = trig.arccos(b1);
	// femur angle
	table.femurAngles[legIndex] = -(long) (legAngle + femurAngle) * 180 / 3141 + 900 + FEMUR_HORN_OFFSET(legIndex);

	// tibia angle
	long a2N = ((((long) (byte) pgm_read_word(&FEMUR_LENGTHS[legIndex]) * (byte) pgm_read_word(&FEMUR_LENGTHS[legIndex])) + ((long) (byte) pgm_read_word(&TIBIA_LENGTHS[legIndex]) * (byte) pgm_read_word(&TIBIA_LENGTHS[legIndex]))) * POW4 - ((long) legLength * legLength));
	long a2D = (2 * (byte) pgm_read_word(&FEMUR_LENGTHS[legIndex]) * (byte) pgm_read_word(&TIBIA_LENGTHS[legIndex]));
	long a2 = trig.arccos(a2N / a2D);
	table.tibiaAngles[legIndex] = -(900 - (long) a2 * 180 / 3141);

	if (legLength < ((byte) pgm_read_word(&FEMUR_LENGTHS[legIndex]) + (byte) pgm_read_word(&TIBIA_LENGTHS[legIndex]) - 30) * POW2) {
		return SOLUTION_FOUND;
	} else {
		if (legLength < ((byte) pgm_read_word(&FEMUR_LENGTHS[legIndex]) + (byte) pgm_read_word(&TIBIA_LENGTHS[legIndex])) * POW2) {
			return SOLUTION_WARN;
		} else {
			return SOLUTION_ERROR;
		}
	}
}
void InverseKinematics::calculateBalanceTilts(BalanceState *out, Gait &gait, PositionTable &table) {
	out->zero();
	for(int i=0; i <= 2; i++) {
		short x = (-table.xPos[i] + gait.gaitXPos[i]);
		short z = table.zPos[i] + gait.gaitZPos[i];
		short y = (table.yPos[i] - (short) pgm_read_word(&INIT_POSY[i])) + gait.gaitYPos[i];
		calculateLegBalance(out, i, x, z, y);
	}

	for(int i=3; i <= 5; i++) {
		short x = table.xPos[i] + gait.gaitXPos[i];
		short z = table.zPos[i] + gait.gaitZPos[i];
		short y = table.yPos[i] - (short) pgm_read_word(&INIT_POSY[i]) + gait.gaitYPos[i];
		calculateLegBalance(out, i, x, z, y);
	}

	finaliseBalanceState(out);
}

void InverseKinematics::calculateLegBalance(BalanceState *out, int index, short posX, short posZ, short posY) {
	// centre of body to feet
	short zOffset = (short) pgm_read_word(&OFFSET_COXA_Z[index]) + posZ;
	short xOffset = (short) pgm_read_word(&OFFSET_COXA_X[index]) + posX;
	short yOffset = 150 + posY;

	out->translationY += (long) posY;
	out->translationZ += (long) zOffset;
	out->translationX += (long) xOffset;

	ATan2Tuple atan2 = trig.atan2(xOffset, zOffset);
	out->balanceY += (atan2.angle * 1800) / 31415;

	// -900 for periodic pwm
	atan2 = trig.atan2(xOffset, yOffset);
	out->balanceZ += ((atan2.angle * 1800) / 31415) - 900;

	atan2 = trig.atan2(zOffset, yOffset);
	out->balanceX += ((atan2.angle * 1800) / 31415) - 900;
}

void InverseKinematics::finaliseBalanceState(BalanceState *out) {
	out->scale(BALANCE_FACTOR);
	out->checkBalancingBounds();
	out->finalise(BALANCE_FACTOR);
}

void InverseKinematics::setRotationOffsets(short rx, short ry, short rz) {
	rotationOffsetX = rx;
	rotationOffsetY = ry;
	rotationOffsetZ = rz;
}

