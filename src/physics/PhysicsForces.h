#ifndef PHYSICS_FORCES_H_
#define PHYSICS_FORCES_H_

#include <systemc.h>
#include "Physics.h"

using namespace gmtl;

class Gravity : public PhysicsComponent
{
public:
	Vec3d gvec;

	Gravity(Vec3d gvec) {
		this->gvec = gvec;
	}

	void update(double delta, PhysicsObject parent) {
		parent.pos_vel += gvec*delta;
	}
};

class Drag : public PhysicsComponent
{
public:
	double density;
	double coeff;
	double area;

	Drag(double density, double coeff, double area) {
		this->density = density;
		this->coeff = coeff;
		this->area = area;
	}

	void update(double delta, PhysicsObject parent) {
		parent.pos_vel = parent.pos_vel + 0.5*density*(parent.pos_vel*parent.pos_vel)*coeff*area*delta;
	}
};

#endif
