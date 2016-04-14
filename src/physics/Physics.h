#ifndef PHYSICS_H_
#define PHYSICS_H_

#include <systemc.h>
#include <cmath>
#include "gmtl/gmtl.h"

using namespace gmtl;
using namespace std;

#define EARTH_GRAVITY 9.80665
#define AIR_DENSITY 1.225

class PhysicsObject;
class PhysicsComponent;

// Physics SystemC Thread
class PhysicsSim : public sc_module
{
public:
	vector<PhysicsObject*> objects;

	// Timing variables
	double delta_ms;
	double time;
	double sim_time;

	// Environment variables
	double density;
	Vec3d gravity;

	// SystemC Variables
	sc_event event_sim_delta;
	sc_event event_sim_trigger;

	SC_HAS_PROCESS(PhysicsSim);

	PhysicsSim(sc_module_name name, double delta_ms, double sim_time) : sc_module(name) {
		SC_THREAD(main);
		this->delta_ms = delta_ms;
		this->sim_time = sim_time;
		this->time = 0;
		this->density = AIR_DENSITY;
		this->gravity = Vec3d(0, 0, -EARTH_GRAVITY);
	}

	PhysicsSim(sc_module_name name, double delta_ms) : PhysicsSim(name, delta_ms, 0) {}

	void addObject(PhysicsObject* p) {
		objects.push_back(p);
	}

	double timeElapsed() {
		return time;
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
	AxisAngled orientation_aa;
public:
	Point3d position;
	Vec3d velocity;
	Quatd orientation;
	Vec3d rotation;
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

	Vec3d upDirection() {
		Vec3d norm(0,0,1);
		xform(norm, orientation, norm);
		return norm;
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

	// Propagate sum of forces to velocity change
	velocity += force*(delta/mass);
	rotation += torque*(delta/mass);

	// Propagate velocities to position/orientation
	position = position + velocity*delta;
	Quatd av;
	set(av, AxisAngled(length(rotation), rotation.mData[0], rotation.mData[1], rotation.mData[2]));
	slerp(orientation, delta, orientation, orientation*av, false);

	// Update axis-angle representation
	set(orientation_aa, orientation);

	// Log physics state
	if(enablelog) {
		//outfile << time << "\t" << position << "\t" << velocity << "\t" << orientation_aa << "\t" << rotation << endl;
		outfile << time << "\t" << position.mData[0] << "\t" << position.mData[1] << "\t" << position.mData[2] << endl;
	}
	time += delta;	// Update total time elapsed

	if(position.mData[2] < 0) {
		sc_stop();
		cout << "Vehicle crashed (Height < 0m)" << endl;
	}
}

#endif
