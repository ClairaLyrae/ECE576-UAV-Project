#ifndef GPS_H_
#define GPS_H_

#include <systemc.h>
#include "../Physics.h"

class GPS : public PhysicsComponent, public sc_module
{
public:
	SC_HAS_PROCESS(GPS);

	GPS(sc_module_name name) : sc_module(name) {
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
