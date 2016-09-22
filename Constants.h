/*
 * Constants.h
 *
 *  Created on: 18 Mar 2016
 *      Author: Bibl
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define	RR			0
#define	RM			1
#define	RF			2
#define	LR			3
#define	LM			4
#define	LF			5

// Mechanical limits of the Right Rear Leg
#define MIN_RR_COXA     -650
#define MAX_RR_COXA     650
#define MIN_RR_FEMUR    -1050
#define MAX_RR_FEMUR    750
#define MIN_RR_TIBIA    -530
#define MAX_RR_TIBIA    900
// Mechanical limits of the Right Middle Leg
#define MIN_RM_COXA     -650
#define MAX_RM_COXA     650
#define MIN_RM_FEMUR    -1050
#define MAX_RM_FEMUR    750
#define MIN_RM_TIBIA    -530
#define MAX_RM_TIBIA    900
// Mechanical limits of the Right Front Leg
#define MIN_RF_COXA     -650
#define MAX_RF_COXA     650
#define MIN_RF_FEMUR    -1050
#define MAX_RF_FEMUR    750
#define MIN_RF_TIBIA    -530
#define MAX_RF_TIBIA    900
// Mechanical limits of the Left Rear Leg
#define MIN_LR_COXA     -650
#define MAX_LR_COXA     650
#define MIN_LR_FEMUR    -1050
#define MAX_LR_FEMUR    750
#define MIN_LR_TIBIA    -530
#define MAX_LR_TIBIA    900
// Mechanical limits of the Left Middle Leg
#define MIN_LM_COXA     -650
#define MAX_LM_COXA     650
#define MIN_LM_FEMUR    -1050
#define MAX_LM_FEMUR    750
#define MIN_LM_TIBIA    -530
#define MAX_LM_TIBIA    900
// Mechanical limits of the Left Front Leg
#define MIN_LF_COXA     -650
#define MAX_LF_COXA     650
#define MIN_LF_FEMUR    -1050
#define MAX_LF_FEMUR    750
#define MIN_LF_TIBIA    -530
#define MAX_LF_TIBIA    900

#define PS2_DAT         11 // brown
#define PS2_CMD         10 // pink
#define PS2_ATT         9  // yellow
#define PS2_CLK         8  // blue

const short MIN_COXA[]   PROGMEM = {MIN_RR_COXA, MIN_RM_COXA, MIN_RF_COXA, MIN_LR_COXA, MIN_LM_COXA, MIN_LF_COXA};
const short MAX_COXA[]   PROGMEM = {MAX_RR_COXA, MAX_RM_COXA, MAX_RF_COXA, MAX_LR_COXA, MAX_LM_COXA, MAX_LF_COXA};
const short MIN_FEMUR[]  PROGMEM = {MIN_RR_FEMUR, MIN_RM_FEMUR, MIN_RF_FEMUR, MIN_LR_FEMUR, MIN_LM_FEMUR, MIN_LF_FEMUR};
const short MAX_FEMUR[]  PROGMEM = {MAX_RR_FEMUR, MAX_RM_FEMUR, MAX_RF_FEMUR, MAX_LR_FEMUR, MAX_LM_FEMUR, MAX_LF_FEMUR};
const short MIN_TIBIA[]  PROGMEM = {MIN_RR_TIBIA, MIN_RM_TIBIA, MIN_RF_TIBIA, MIN_LR_TIBIA, MIN_LM_TIBIA, MIN_LF_TIBIA};
const short MAX_TIBIA[]  PROGMEM = {MAX_RR_TIBIA, MAX_RM_TIBIA, MAX_RF_TIBIA, MAX_LR_TIBIA, MAX_LM_TIBIA, MAX_LF_TIBIA};

#define	POW1		10
#define	POW2		100
#define	POW4		10000
#define	POW6		1000000

#define STANDARD_COXA_LENGTH     29
#define STANDARD_FEMUR_LENGTH    57
#define STANDARD_TIBIA_LENGTH    141

#define INIT_XZ	      111
#define INIT_XZ_COS60  56    // COS(60) = .5
#define INIT_HZ_SIN60  96    // sin(60) = .866
#define INIT_Y		   65    // 30

// RR
#define INIT_RR_POSX     INIT_XZ_COS60
#define INIT_RR_POSY     INIT_Y
#define INIT_RR_POSZ     INIT_HZ_SIN60
// RM
#define INIT_RM_POSX     INIT_XZ
#define INIT_RM_POSY     INIT_Y
#define INIT_RM_POSZ     0
// RF
#define INIT_RF_POSX     INIT_XZ_COS60
#define INIT_RF_POSY     INIT_Y
#define INIT_RF_POSZ     -INIT_HZ_SIN60
// LR
#define INIT_LR_POSX     INIT_XZ_COS60
#define INIT_LR_POSY     INIT_Y
#define INIT_LR_POSZ     INIT_HZ_SIN60
// LM
#define INIT_LM_POSX     INIT_XZ
#define INIT_LM_POSY     INIT_Y
#define INIT_LM_POSZ     0
// LF
#define INIT_LF_POSX     INIT_XZ_COS60
#define INIT_LF_POSY     INIT_Y
#define INIT_LF_POSZ     -INIT_HZ_SIN60

const short INIT_POSX[] PROGMEM = { INIT_RR_POSX, INIT_RM_POSX, INIT_RF_POSX, INIT_LR_POSX, INIT_LM_POSX, INIT_LF_POSX };
const short INIT_POSY[] PROGMEM = { INIT_RR_POSY, INIT_RM_POSY, INIT_RF_POSY, INIT_LR_POSY, INIT_LM_POSY, INIT_LF_POSY };
const short INIT_POSZ[] PROGMEM = { INIT_RR_POSZ, INIT_RM_POSZ, INIT_RF_POSZ, INIT_LR_POSZ, INIT_LM_POSZ, INIT_LF_POSZ };

// angle offsets
#define FEMUR_HORN_OFFSET(LEGINDEX)  (0)
#define ANGLE_RR_COXA   -600
#define ANGLE_RM_COXA    0
#define ANGLE_RF_COXA    600
#define ANGLE_LR_COXA    -600
#define ANGLE_LM_COXA    0
#define ANGLE_LF_COXA    600
const short COXA_ANGLES[] PROGMEM = { ANGLE_RR_COXA, ANGLE_RM_COXA, ANGLE_RF_COXA, ANGLE_LR_COXA, ANGLE_LM_COXA, ANGLE_LF_COXA };

// body segment offsets
#define OFFSET_COXA_RR_X      -69
#define OFFSET_COXA_RR_Z      119
#define OFFSET_COXA_RM_X      -138
#define OFFSET_COXA_RM_Z      0
#define OFFSET_COXA_RF_X      -69
#define OFFSET_COXA_RF_Z      -119

#define OFFSET_COXA_LR_X      69
#define OFFSET_COXA_LR_Z      119
#define OFFSET_COXA_LM_X      138
#define OFFSET_COXA_LM_Z      0
#define OFFSET_COXA_LF_X      69
#define OFFSET_COXA_LF_Z      -119
const short OFFSET_COXA_X[] PROGMEM = { OFFSET_COXA_RR_X, OFFSET_COXA_RM_X, OFFSET_COXA_RF_X, OFFSET_COXA_LR_X, OFFSET_COXA_LM_X, OFFSET_COXA_LF_X };
const short OFFSET_COXA_Z[] PROGMEM = { OFFSET_COXA_RR_Z, OFFSET_COXA_RM_Z, OFFSET_COXA_RF_Z, OFFSET_COXA_LR_Z, OFFSET_COXA_LM_Z, OFFSET_COXA_LF_Z };

const int PIN_LAYOUT[] PROGMEM = {16, 17, 18, 21, 22, 23, 25, 26, 27, 1, 2, 3, 8, 9, 10, 13,14,15};

const short COXA_LENGTHS[]  PROGMEM = {STANDARD_COXA_LENGTH, STANDARD_COXA_LENGTH, STANDARD_COXA_LENGTH, STANDARD_COXA_LENGTH, STANDARD_COXA_LENGTH, STANDARD_COXA_LENGTH};
const short FEMUR_LENGTHS[] PROGMEM = {STANDARD_FEMUR_LENGTH, STANDARD_FEMUR_LENGTH, STANDARD_FEMUR_LENGTH, STANDARD_FEMUR_LENGTH, STANDARD_FEMUR_LENGTH, STANDARD_FEMUR_LENGTH};
const short TIBIA_LENGTHS[] PROGMEM = {STANDARD_TIBIA_LENGTH, STANDARD_TIBIA_LENGTH, STANDARD_TIBIA_LENGTH, STANDARD_TIBIA_LENGTH, STANDARD_TIBIA_LENGTH, STANDARD_TIBIA_LENGTH};

#define SOLUTION_FOUND 0x1
#define SOLUTION_ERROR 0x2
#define SOLUTION_WARN  0x4

#define MAX_BODY_Y_OFFSET 125
#define PWM_DIV        991
#define PF_ADJUST      592
#define DEADZONE       4

#endif /* CONSTANTS_H_ */
