#ifndef PHYSICS_COMP_H_
#define PHYSICS_COMP_H_

#include <systemc.h>
#include "Physics.h"

using namespace gmtl;

class Motor : public PhysicsComponent, public sc_module
{
public:
	Vec3d position;
	double thrustLevel;

	SC_HAS_PROCESS(Motor);

	Motor(sc_module_name name, Vec3d position) : sc_module(name) {
		SC_THREAD(main);
		this->position = position;
	}

	void main() {
		// Process PWM signals
	}

	void update(double delta, PhysicsObject parent) {
		// Update physics object velocities based on motor thrust curve
		// Apply torque and lin vel
	}
};

#endif
