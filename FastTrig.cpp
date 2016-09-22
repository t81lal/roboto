/*
 * FastTrig.cpp
 *
 *  Created on: 18 Mar 2016
 *      Author: Bibl
 */

#include <Arduino.h>
#include "FastTrig.h"
#include "Constants.h"

static const byte ACOS_TABLE[] PROGMEM = {255, 254, 252, 251, 250, 249, 247, 246, 245, 243, 242, 241, 240, 238, 237, 236, 234, 233, 232, 231, 229, 228, 227, 225, 224, 223, 221, 220, 219, 217, 216, 215,
		214, 212, 211, 210, 208, 207, 206, 204, 203, 201, 200, 199, 197, 196, 195, 193, 192, 190, 189, 188, 186, 185, 183, 182, 181, 179, 178, 176, 175, 173, 172, 170, 169, 167, 166, 164, 163, 161,
		160, 158, 157, 155, 154, 152, 150, 149, 147, 146, 144, 142, 141, 139, 137, 135, 134, 132, 130, 128, 127, 125, 123, 121, 119, 117, 115, 113, 111, 109, 107, 105, 103, 101, 98, 96, 94, 92, 89,
		87, 84, 81, 79, 76, 73, 73, 73, 72, 72, 72, 71, 71, 71, 70, 70, 70, 70, 69, 69, 69, 68, 68, 68, 67, 67, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 63, 63, 63, 62, 62, 62, 61, 61, 61, 60, 60, 59,
		59, 59, 58, 58, 58, 57, 57, 57, 56, 56, 55, 55, 55, 54, 54, 53, 53, 53, 52, 52, 51, 51, 51, 50, 50, 49, 49, 48, 48, 47, 47, 47, 46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 41, 41, 40, 40, 39, 39,
		38, 37, 37, 36, 36, 35, 34, 34, 33, 33, 32, 31, 31, 30, 29, 28, 28, 27, 26, 25, 24, 23, 23, 23, 23, 22, 22, 22, 22, 21, 21, 21, 21, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17, 16,
		16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 6, 6, 5, 3, 0};

static const word SIN_TABLE[] PROGMEM = {0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564, 1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419,
		2503, 2588, 2672, 2756, 2840, 2923, 3007, 3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383, 4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999,
		5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664, 5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819, 6883, 6946, 7009, 7071, 7132, 7193,
		7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826, 7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660, 8703, 8746, 8788, 8829,
		8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 9762, 9781,
		9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969, 9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000};


FastTrig::FastTrig() {
}

FastTrig::~FastTrig() {
}

AngleTuple FastTrig::sincos(short theta) {
	short abs;
	if (theta < 0) {
		abs = theta * -1;
	} else {
		abs = theta;
	}

	if (theta < 0) {
		theta = 3600 - (abs - (3600 * (abs / 3600)));
	} else {
		theta = abs - (3600 * (abs / 3600));
	}

	short sin = 0;
	short cos = 0;

	if (theta >= 0 && theta <= 900) {
		// 0 to 90
		sin = pgm_read_word(&SIN_TABLE[theta / 5]);
		cos = pgm_read_word(&SIN_TABLE[(900 - (theta)) / 5]);
	} else if (theta > 900 && theta <= 1800) {
		// 90 to 180
		sin = pgm_read_word(&SIN_TABLE[(900 - (theta - 900)) / 5]);
		cos = -pgm_read_word(&SIN_TABLE[(theta - 900) / 5]);
	} else if (theta > 1800 && theta <= 2700) {
		// 180 to 270
		sin = -pgm_read_word(&SIN_TABLE[(theta - 1800) / 5]);
		cos = -pgm_read_word(&SIN_TABLE[(2700 - theta) / 5]);
	} else if (theta > 2700 && theta <= 3600) {
		// 270 to 360
		sin = -pgm_read_word(&SIN_TABLE[(3600 - theta) / 5]);
		cos = pgm_read_word(&SIN_TABLE[(theta - 2700) / 5]);
	}
	return AngleTuple(sin, cos);
}
long FastTrig::arccos(short cos) {
	boolean negative/*:1*/;
	if (cos < 0) {
		cos = -cos;
		negative = 1;
	} else
		negative = 0;

	cos = min(cos, POW4);

	long angle = 0;

	// 616=acos resolution (pi/2/255)
	if ((cos >= 0) && (cos < 9000)) {
		angle = (byte) pgm_read_byte(&ACOS_TABLE[cos / 79]);
		angle = ((long) angle * 616) / POW1;
	} else if ((cos >= 9000) && (cos < 9900)) {
		angle = (byte) pgm_read_byte(&ACOS_TABLE[(cos - 9000) / 8 + 114]);
		angle = (long) ((long) angle * 616) / POW1;
	} else if ((cos >= 9900) && (cos <= 10000)) {
		angle = (byte) pgm_read_byte(&ACOS_TABLE[(cos - 9900) / 2 + 227]);
		angle = (long) ((long) angle * 616) / POW1;
	}

	if (negative) {
		angle = 31416 - angle;
	}

	return angle;
}

unsigned long FastTrig::isqrt32(unsigned long n) {
	unsigned long root = 0;
	unsigned long remainder = n;
	unsigned long place = 0x40000000;

	while (place > remainder)
		place = place >> 2;
	while (place) {
		if (remainder >= root + place) {
			remainder = remainder - root - place;
			root = root + (place << 1);
		}
		root = root >> 1;
		place = place >> 2;
	}
	return root;
}

ATan2Tuple FastTrig::atan2(short x, short y) {
	short hyp = isqrt32(((long) x * x * POW4) + ((long) y * y * POW4));
	// radians
	long angle = arccos(((long) x * (long) POW6) / (long) hyp);

	if (y < 0) {
		angle = -angle;
	}

	return ATan2Tuple(angle, hyp);
}
