#ifndef MOTOR_H_
#define MOTOR_H_

#include <cmath>
#include <systemc.h>
#include "gmtl/gmtl.h"
#include "../Physics.h"
#include "../../comm/PWM.h"

using namespace gmtl;
using namespace std;

#define M_PI_4 0.78539816339

class Motor : public PhysicsComponent, public sc_module
{
public:
	Vec3d position;
	double thrustLevel;		// 0 to 1 thrust
	double propDiameter;
	double propPitch;
	double maxRPS;

	//sc_port<PWM_in> pwm;

	SC_HAS_PROCESS(Motor);

	Motor(sc_module_name name, Vec3d position, double propDiameter, double propPitch, double maxRPM) : sc_module(name) {
		SC_THREAD(main);
		this->position = position;
		this->propPitch = propPitch;
		this->propDiameter = propDiameter;
		this->maxRPS = maxRPM/60.0;
		this->thrustLevel = 0;
	}

	void main() {
//		while(true) {
//			thrustLevel = pwm->listenPWM();
//		}
	}

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Get vehicle up direction
		Vec3d norm = parent.upDirection();

		// Find velocity along motor up direction
		double mvel = dot(parent.velocity, norm);

		// Find motor RPS from current thrust PWM level
		double rps = maxRPS*thrustLevel;

		// Find resulting force along motor direction
		double t1 = M_PI_4*sim.density*pow(propDiameter, 2)*pow(propDiameter/(3.29546*propPitch), 1.5);
		double t2 = t1*rps*propPitch*(rps*propPitch - mvel);
		Vec3d thrust = norm*t2;
		parent.force += thrust;
		cout << "Motor Thrust: " << thrust << ", at vel of " << mvel << endl;

		// Find the torque from motor lever action
		Vec3d thrust_t;
		cross(thrust_t, position, thrust);
		xform(thrust_t, parent.orientation, thrust_t);
		parent.torque += thrust_t;

		// Find the torque from propeller rotation (ignored due to symmetry)
	}
};

#endif
