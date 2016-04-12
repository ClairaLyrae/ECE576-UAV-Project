#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include <systemc.h>
#include "PhysicsObjects.h"

using namespace gmtl;

class Magnetometer : public PhysicsComponent, public sc_module
{
public:
	SC_HAS_PROCESS(Magnetometer);

	Magnetometer(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
	}

	void main() {
		// Respond to I2C bus
	}

	void update(double delta, PhysicsObject &parent) {
		// Update sensor data from physics object
	}
};

#endif
