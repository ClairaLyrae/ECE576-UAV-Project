#ifndef PHYSICS_H_
#define PHYSICS_H_

#include <systemc.h>
#include "gmtl.h"

using namespace gmtl;

// Physics Object
class PhysicsObject
{
public:
	Vec3d pos;
	Vec3d pos_vel;
	AxisAngled rot;
	AxisAngled rot_vel;
	double mass;

	std::vector<PhysicsComponent> components;

	PhysicsObject(double mass) {
		this->mass = mass;
	}

	void addComponent(PhysicsComponent p) {
		components.push_back(p);
	}

	void update(double delta) {
		for(PhysicsComponent p : components) {
			p.update(delta, *this);
		}

		// Propagate velocities to position/orientation
		pos = pos + pos_vel*delta;
		rot = rot + rot_vel*delta;
	}
};

// Physics Component
class PhysicsComponent
{
public:
	virtual void update(double delta, PhysicsObject parent);
};

// Physics SystemC Thread
class Physics : public sc_module
{
public:
	std::vector<PhysicsObject> objects;
	double delta_ms;

	SC_HAS_PROCESS(Physics);

	Physics(sc_module_name name, double delta_ms) : sc_module(name) {
		SC_THREAD(main);
		this->delta_ms = delta_ms;
	}

	void addObject(PhysicsObject p) {
		objects.push_back(p);
	}

	void main() {
		while(true) {
			wait(delta_ms, SC_MS);
			for(PhysicsObject p : objects) {
				p.update(delta_ms*0.001);
			}
		}
		return;
	}
};

#endif
