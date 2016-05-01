#ifndef ALT_EST_H_
#define ALT_EST_H_

#include "SensorData.h"

#define COMP_FILTER_A 1.0
#define COMP_FILTER_B 1.0

class AltitudeEstimator
{
private:
	uint8_t hEstimateInit;
	float   accelZ;
	float   hEstimationError;
public:
	float   h;
	float   hDot;

	AltitudeEstimator() {
		hDot = 0;
		hEstimationError = 0;
		hEstimateInit = false;
	}

	void update(double zbaro, double zaccel, float dt)
	{
	    if (!hEstimateInit) {
	    	h = zbaro;
	    	hEstimateInit = true;
	    } else {
	    	accelZ = -zaccel + COMP_FILTER_B * hEstimationError;
	        hDot += accelZ * dt;
	        h += (hDot + COMP_FILTER_A * hEstimationError) * dt;
	        hEstimationError = zbaro - h;
	    }
	}
};

#endif
