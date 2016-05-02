#ifndef MOTOR_H_
#define MOTOR_H_

#include <cmath>
#include <systemc.h>
#include "gmtl/gmtl.h"
#include "../Physics.h"
#include "../../PWM.h"

using namespace gmtl;
using namespace std;

#define M_PI 3.14159265359
#define M_PI_4 0.78539816339

class Motor : public PhysicsComponent, public sc_module
{
public:
	Vec3d position;
	double thrustLevel;		// 0 to 1 thrust
	double propDiameter;
	double propPitch;
	double maxRPS;
	double stallTorque;
	bool cw;

	sc_port<PWM_in> pwm_in;

	SC_HAS_PROCESS(Motor);

	Motor(sc_module_name name, Vec3d position, double propDiameter, double propPitch, double maxRPM, double maxTorque, bool cw) : sc_module(name) {
		SC_THREAD(main);
		this->position = position;
		this->propPitch = propPitch;
		this->propDiameter = propDiameter;
		this->maxRPS = maxRPM/60.0;
		this->thrustLevel = 0;
		this->cw = cw;
		this->stallTorque = maxTorque;
	}

	void main() {
		while(true) {
			thrustLevel = pwm_in->listenPWM();
			if(thrustLevel > 1.0)
				thrustLevel = 1.0;
			else if(thrustLevel < 0.0)
				thrustLevel = 0.0;
		}
	}

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Get vehicle up direction
		Vec3d norm = parent.orientationNormal();

		// Find velocity along motor up direction
		double mvel = dot(parent.velocity, norm);

		// Find motor RPS from current thrust PWM level
		double rps = maxRPS*thrustLevel;

		// Find resulting force along motor direction
		double t1 = M_PI_4*sim.density*pow(propDiameter, 2)*pow(propDiameter/(3.29546*propPitch), 1.5);
		double t2 = t1*rps*propPitch*(rps*propPitch - mvel);
		Vec3d thrust = norm*t2;
		parent.force += thrust;

		// Find the torque from motor lever action
		Vec3d thrust_t(0,0,t2);
		cross(thrust_t, position, thrust_t);
		xform(thrust_t, parent.orientation, thrust_t);
		parent.torque += thrust_t;

		// Find the torque from propeller rotation
		Vec3d spin_t(0,0,stallTorque*(1 - rps/maxRPS));
		spin_t *= cw ? 1 : -1;
		xform(spin_t, parent.orientation, spin_t);
		parent.torque += spin_t;
	}
};

#endif
