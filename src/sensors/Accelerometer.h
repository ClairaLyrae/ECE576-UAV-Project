#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include <systemc.h>
#include "Physics.h"

using namespace gmtl;

class Accelerometer : public PhysicsComponent, public sc_module
{
public:
	SC_HAS_PROCESS(Accelerometer);

	Accelerometer(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
	}

	void main() {
		// Respond to I2C bus
	}

	void update(double delta, PhysicsObject parent) {
		// Update sensor data from physics object
	}
};

#endif
