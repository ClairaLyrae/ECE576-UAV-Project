#ifndef PHYSICS_H_
#define PHYSICS_H_

#include <systemc.h>
#include <cmath>
#include "gmtl/gmtl.h"
#include "../util/util.h"

using namespace gmtl;
using namespace std;

#define AIR_MOLAR_MASS 0.0289644	// kg/mol
#define TEMP_LAPSE_RATE 0.0065		// K/m
#define GAS_CONSTANT 8.31447		// J/(mol*K)
#define EARTH_GRAVITY 9.80665		// m/s^2
#define AIR_DENSITY 1.225			//
#define STP_TEMP 288.15				// K
#define STP_PRESSURE 101325			// Pa

#define M_PI 3.14159265359
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#define ROLL 0
#define PITCH 1
#define YAW 2

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
	Vec3d magfield;

	SC_HAS_PROCESS(PhysicsSim);

	PhysicsSim(sc_module_name name, double delta_ms, double sim_time) : sc_module(name) {
		SC_THREAD(main);
		this->delta_ms = delta_ms;
		this->sim_time = sim_time;
		this->time = 0;
		this->density = AIR_DENSITY;
		this->gravity = Vec3d(0, 0, -EARTH_GRAVITY);
		this->magfield = Vec3d(0.2416, 0.0429, 0.4037);	// Magnetic field at university of arizona
		this->temperature = STP_TEMP;
	}

	PhysicsSim(sc_module_name name, double delta_ms) : PhysicsSim(name, delta_ms, 0) {}

	void addObject(PhysicsObject* p) {
		objects.push_back(p);
	}

	double timeElapsed() {
		return time;
	}

	// Returns pressure in Pa
	double getPressure(double altitude) {
		double A = (EARTH_GRAVITY*AIR_MOLAR_MASS)/(GAS_CONSTANT*TEMP_LAPSE_RATE);
		return STP_PRESSURE*pow(getTemperature(altitude)/STP_TEMP, A);
	}

	// Returns gravity in m/s^2
	Vec3d getGravity() {
		return gravity;
	}

	// Returns magnetic field in Gauss
	Vec3d getMagneticField() {
		return magfield;
	}

	// Returns density in kg/m^3
	double getDensity(double altitude) {
		return (getPressure(altitude)*AIR_MOLAR_MASS)/(GAS_CONSTANT*getTemperature(altitude));
	}

	// Returns temperature in K
	double getTemperature(double altitude) {
		return STP_TEMP - altitude*TEMP_LAPSE_RATE;
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

	// Utility components
	Vec3d orientation_vec_up;
	Vec3d orientation_vec_forward;
	AxisAngled orientation_aa;
	AxisAngled rotation_aa;
	Quatd rotation_q;
public:
	// Translation Components
	Point3d position;
	Vec3d velocity;
	Vec3d acceleration;

	// Rotation components
	Quatd orientation;
	Vec3d rotation;
	Vec3d angularAcceleration;

	// Attitude components
	Vec3d attitude;
	Vec3d attitude_rate;

	double dimensions[3];
	double mass;

	// Force components
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
		return orientation_vec_up;
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
		if((int)(time*1000)%1000 == 0)
			cout << "Physics time (t=" << time << "s)" << endl;
		event_sim_delta.notify();
	}
	cout << "Physics simulation end (t=" << time << "s)" << endl;
	sc_stop();
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

	// Update miscellanious representations
	set(orientation_aa, orientation);
	orientation_vec_up.set(0,0,1);
	xform(orientation_vec_up, orientation, orientation_vec_up);
	orientation_vec_forward.set(1,0,0);
	xform(orientation_vec_forward, orientation, orientation_vec_forward);
	toAttitude(attitude, orientation);
	toAttitude(attitude_rate, rotation_q);

	// Log physics state
	if(enablelog) {
		//outfile << time << "\t" << position << "\t" << orientation << "\t" << attitude << "\t" << attitude_rate << endl;
		outfile << time << "\t";
		outfile << position[XAXIS] << "\t" << position[YAXIS] << "\t" << position[ZAXIS] << "\t";
		outfile << velocity[XAXIS] << "\t" << velocity[YAXIS] << "\t" << velocity[ZAXIS] << "\t";
		outfile << attitude[ROLL] << "\t" << attitude[PITCH] << "\t" << attitude[YAW] << "\t";
		outfile << attitude_rate[ROLL] << "\t" << attitude_rate[PITCH] << "\t" << attitude_rate[YAW] << "\t";
		outfile << orientation_vec_up[XAXIS] << "\t" << orientation_vec_up[YAXIS] << "\t" << orientation_vec_up[ZAXIS] << "\t";
		outfile << orientation_vec_forward[XAXIS] << "\t" << orientation_vec_forward[YAXIS] << "\t" << orientation_vec_forward[ZAXIS] << "\t";
		outfile << endl;
	}
	time += delta;	// Update total time elapsed

	if(position.mData[2] < 0) {
		sc_stop();
		cout << "Vehicle crashed (Height < 0m)" << endl;
	}
}

#endif
