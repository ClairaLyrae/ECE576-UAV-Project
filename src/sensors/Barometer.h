#ifndef BAROMETER_H_
#define BAROMETER_H_

#include <systemc.h>
#include "Physics.h"

using namespace gmtl;

class Barometer : public PhysicsComponent, public sc_module
{
public:
	SC_HAS_PROCESS(Barometer);

	Barometer(sc_module_name name) : sc_module(name) {
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
