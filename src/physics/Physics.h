#ifndef PHYSICS_H_
#define PHYSICS_H_

#include <systemc.h>
#include <cmath>
#include "gmtl/gmtl.h"

using namespace gmtl;
using namespace std;

#define AIR_MOLAR_MASS 0.0289644
#define TEMP_LAPSE_RATE 0.0065
#define GAS_CONSTANT 8.31447
#define EARTH_GRAVITY 9.80665
#define AIR_DENSITY 1.225
#define STP_TEMP 15
#define STP_PRESSURE 101325

class PhysicsObject;
class PhysicsComponent;

// Physics SystemC Thread
class PhysicsSim : public sc_module
{
private:
	vector<PhysicsObject*> objects;

	double delta_ms;
	double sim_time;
	double time;
	sc_event event_sim_delta;
	sc_event event_sim_trigger;
public:
	// Timing variables

	// Environment variables
	double temperature;
	double density;
	Vec3d gravity;

	SC_HAS_PROCESS(PhysicsSim);

	PhysicsSim(sc_module_name name, double delta_ms, double sim_time) : sc_module(name) {
		SC_THREAD(main);
		this->delta_ms = delta_ms;
		this->sim_time = sim_time;
		this->time = 0;
		this->density = AIR_DENSITY;
		this->gravity = Vec3d(0, 0, -EARTH_GRAVITY);
		this->temperature = STP_TEMP;
	}

	PhysicsSim(sc_module_name name, double delta_ms) : PhysicsSim(name, delta_ms, 0) {}

	void addObject(PhysicsObject* p) {
		objects.push_back(p);
	}

	double timeElapsed() {
		return time;
	}

	double pressureAtAltitude(double z) {
		double A = (EARTH_GRAVITY*AIR_MOLAR_MASS)/(GAS_CONSTANT*TEMP_LAPSE_RATE);
		double B = TEMP_LAPSE_RATE/STP_TEMP;
		return STP_PRESSURE*pow(1 - ((TEMP_LAPSE_RATE*z)/STP_TEMP), A);
	}

	double densityAtAltitude(double z) {
		double A = (EARTH_GRAVITY*AIR_MOLAR_MASS)/(GAS_CONSTANT*TEMP_LAPSE_RATE);
		return STP_PRESSURE*pow(STP_TEMP/(STP_TEMP + (TEMP_LAPSE_RATE*z)), 1 + A);
	}

	void main();
};

// Physics Object
class PhysicsObject
{
private:
	ofstream outfile;
	string outfilename;
	bool enablelog;
	double time;
	Vec3d orientation_vec;
	AxisAngled orientation_aa;
	AxisAngled rotation_aa;
	Quatd rotation_q;
public:
	Point3d position;
	Vec3d velocity;
	Vec3d acceleration;
	Quatd orientation;
	Vec3d rotation;
	Vec3d angularAcceleration;
	double dimensions[3];
	double mass;

	Vec3d force;
	Vec3d torque;

	vector<PhysicsComponent*> components;

	PhysicsObject(double mass, float length, float width, float height) {
		this->mass = mass;
		this->enablelog = false;
		this->time = 0;
		this->dimensions[0] = length;
		this->dimensions[1] = width;
		this->dimensions[2] = height;
	}

	Quatd rotationQuaternion() {
		return rotation_q;
	}

	AxisAngled rotationAxisAngle() {
		return rotation_aa;
	}

	AxisAngled orientationAxisAngle() {
		return orientation_aa;
	}

	Vec3d orientationNormal() {
		return orientation_vec;
	}

	bool enableLog(string ofile) {
		this->outfilename = ofile;
    	outfile.open(outfilename.c_str());
		enablelog = true;
		if(!outfile.good()) {
			enablelog = false;
		}
		return enablelog;
	}

	void disableLog() {
		if(enablelog) {
			outfile.flush();
			outfile.close();
		}
		enablelog = false;
	}

	void addComponent(PhysicsComponent* p) {
		components.push_back(p);
	}

	void update(double delta, PhysicsSim &sim);

	~PhysicsObject() {
		disableLog();
	}
};

// Physics Component
class PhysicsComponent
{
public:
    virtual ~PhysicsComponent() {};
	virtual void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {};
};

void PhysicsSim::main() {
	while(sim_time == 0 || time < sim_time) {
		wait(delta_ms, SC_MS);
		for(PhysicsObject* p : objects) {
			p->update(delta_ms*0.001, *this);
		}
		time += delta_ms*0.001;
		event_sim_delta.notify();
	}
	return;
}

void PhysicsObject::update(double delta, PhysicsSim &sim) {
	// Reset forces for this physics step
	force.set(0,0,0);
	torque.set(0,0,0);

	// Update components and accumulate forces
	for(PhysicsComponent* p : components) {
		p->update(delta, sim, *this);
	}
	force += sim.gravity*mass;	// Gravity force

	// Propagate sum of forces to acceleration
	acceleration = force/mass;
	angularAcceleration = torque/mass;

	// Propagate acceleration to velocities
	velocity += acceleration*delta;
	rotation += angularAcceleration*delta;

	// Propagate velocities to position/orientation
	position += velocity*delta;
	Vec3d rnorm(rotation);
	normalize(rnorm);
	rotation_aa.set(length(rotation), rnorm.mData[0], rnorm.mData[1], rnorm.mData[2]);
	set(rotation_q, rotation_aa);
	normalize(rotation_q);
	slerp(orientation, delta, orientation, orientation*rotation_q, false);
	normalize(orientation);

	// Update axis-angle/vector representation
	set(orientation_aa, orientation);
	orientation_vec.set(0,0,1);
	xform(orientation_vec, orientation, orientation_vec);

	// Log physics state
	if(enablelog) {
		//outfile << time << "\t" << position << "\t" << velocity << "\t" << orientation_aa << "\t" << rotation << force << torque << endl;
		outfile << time << "\t" << position.mData[0] << "\t" << position.mData[1] << "\t" << position.mData[2] << "\t" << orientation_vec.mData[0] << "\t" << orientation_vec.mData[1] << "\t" << orientation_vec.mData[2] << endl;
	}
	time += delta;	// Update total time elapsed

	if(position.mData[2] < 0) {
		sc_stop();
		cout << "Vehicle crashed (Height < 0m)" << endl;
	}
}

#endif
