#ifndef UTIL_H_
#define UTIL_H_

#include "gmtl/gmtl.h"

#define ROLL 0
#define PITCH 1
#define YAW 2

using namespace gmtl;

float halfToFloat(uint16_t h) {
	uint32_t temp;
	temp = ((h&0x8000)<<16) | (((h&0x7c00)+0x1C000)<<13) | ((h&0x03FF)<<13);
	return *((float*)&temp);
}

uint16_t floatToHalf(float f) {
	uint32_t x = *((uint32_t*)&f);
	return (uint16_t)(((x>>16)&0x8000)|((((x&0x7f800000)-0x38000000)>>13)&0x7c00)|((x>>13)&0x03ff));
}

Vec3d &toAttitude(Vec3d &result, Quatd &q)
{
    double q0q0 = q[0]*q[0];
    double q0q1 = q[0]*q[1];
    double q0q2 = q[0]*q[2];
    double q0q3 = q[0]*q[3];
    double q1q1 = q[1]*q[1];
    double q1q2 = q[1]*q[2];
    double q1q3 = q[1]*q[3];
    double q2q2 = q[2]*q[2];
    double q2q3 = q[2]*q[3];
    double q3q3 = q[3]*q[3];

	result[ROLL] = atan2(2*(q1q2 + q0q3), q3q3 + q2q2 - q0q0 - q1q1);
	result[PITCH] = -asin(2*(q1q3 - q0q2));
	result[YAW] = atan2(2*(q0q1 + q2q3), q0q0 + q3q3 - q1q1 - q2q2);
    return result;
}

class NoiseGenerator {
private:
    bool n2_cached;
    double x, y, r, n1, n2, result;
public:
	double mean;
	double stddev;

	NoiseGenerator(double mean, double stddev) {
		this->mean = mean;
		this->stddev = stddev;
		n2 = 0.0;
		n2_cached = false;
	}

	double generate()
	{
	    if (!n2_cached)
	    {
	        do {
	            x = 2.0*((double)rand()/RAND_MAX) - 1.0;
	            y = 2.0*((double)rand()/RAND_MAX) - 1.0;
	            r = x*x + y*y;
	        } while (r == 0.0 || r > 1.0);
			r = sqrt(-2.0*log(r)/r);
			n1 = x*r;
			n2 = y*r;
			result = n1*stddev + mean;
			n2_cached = true;
			return result;
	    } else {
	        n2_cached = false;
	        return n2*stddev + mean;
	    }
	}
};


#endif
