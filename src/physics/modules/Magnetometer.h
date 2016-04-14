#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include <systemc.h>
#include "../Physics.h"

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

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Update sensor data from physics object
	}
};

#endif
