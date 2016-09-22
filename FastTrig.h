/*
 * FastTrig.h
 *
 *  Created on: 18 Mar 2016
 *      Author: Bibl
 */

#ifndef FASTTRIG_H_
#define FASTTRIG_H_

#include <Arduino.h>

struct AngleTuple {
	short sin;
	short cos;

	AngleTuple(short _sin, short _cos) : sin(_sin), cos(_cos) {
	}
};

struct ATan2Tuple {
	long angle;
	short hyp;

	ATan2Tuple(long _angle, short _hyp) : angle(_angle), hyp(_hyp) {
	}
};

class FastTrig {
public:
	FastTrig();
	virtual ~FastTrig();

	AngleTuple sincos(short theta);
	long arccos(short cval);
	ATan2Tuple atan2(short atanX, short atanY);
	unsigned long isqrt32(unsigned long n);
};

#endif /* FASTTRIG_H_ */
