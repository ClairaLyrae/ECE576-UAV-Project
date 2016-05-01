#ifndef GPS_H_
#define GPS_H_

#include <systemc.h>
#include "../Physics.h"
#include "../../I2C.h"

class A2235H : public PhysicsComponent, public sc_module
{
private:
	unsigned registers[128];
public:
	const unsigned I2C_ADDRESS = 53;
	sc_port<i2c_slv_if> i2c_slv;

	SC_HAS_PROCESS(A2235H);

	A2235H(sc_module_name name) : sc_module(name) {
		//SC_THREAD(main);
	}

	void main() {
	}

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Update sensor data from physics object
		// TODO Calculate GPS values
	}
};

#endif
