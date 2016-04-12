#ifndef PHYSICS_FORCE_H_
#define PHYSICS_FORCE_H_

#include <systemc.h>
#include "PhysicsObjects.h"

using namespace gmtl;

class Gravity : public PhysicsComponent
{
public:
	Vec3d gvec;

	Gravity(Vec3d gvec) {
		this->gvec = gvec;
	}

	void update(double delta, PhysicsObject &parent) {
		parent.velocity += gvec*delta;
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

	void update(double delta, PhysicsObject &parent) {

	}
};

#endif
