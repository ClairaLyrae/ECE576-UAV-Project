#ifndef AERO_DRAG_H_
#define AERO_DRAG_H_

#include <cmath>
#include "gmtl/gmtl.h"
#include "../Physics.h"

using namespace gmtl;
using namespace std;

class Aerodynamics : public PhysicsComponent
{
public:
	double drag_coeff;

	Aerodynamics(double cdrag) {
		this->drag_coeff = cdrag;
	}

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Velocity normal
		Vec3d vnorm(parent.velocity);
		normalize(vnorm);

		// Cross sectional area of object
		double area = parent.dimensions[0]*parent.dimensions[1];

		// Find linear drag force
		double drag = -0.5*drag_coeff*area*sim.density*lengthSquared(parent.velocity);
		Vec3d dragforce = vnorm*drag;

		// Find drag torque

		// Add forces to object
		parent.force = parent.force + dragforce;
	}
};

#endif
