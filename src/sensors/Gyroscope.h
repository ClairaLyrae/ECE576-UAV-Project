#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#include <systemc.h>
#include "Physics.h"

using namespace gmtl;

class Gyroscope : public PhysicsComponent, public sc_module
{
public:
	SC_HAS_PROCESS(Gyroscope);

	Gyroscope(sc_module_name name) : sc_module(name) {
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
