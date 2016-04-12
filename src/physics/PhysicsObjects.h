#ifndef PHYSICS_OBJ_H_
#define PHYSICS_OBJ_H_

#include <systemc.h>
#include "gmtl/gmtl.h"

using namespace gmtl;
using namespace std;

class PhysicsObject;

// Physics Component
class PhysicsComponent
{
public:
    virtual ~PhysicsComponent() {};
	virtual void update(double delta, PhysicsObject &parent) {};
};

// Physics Object
class PhysicsObject
{
private:
	ofstream outfile;
	string outfilename;
	bool enablelog;
	double time;
public:
	Point3d position;
	Vec3d velocity;
	Quatd orientation;
	Quatd rotation;
	double dimensions[3];
	double mass;

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

	void update(double delta) {
		for(PhysicsComponent* p : components) {
			p->update(delta, *this);
		}

		// Propagate velocities to position/orientation
		position = position + velocity*delta;
		slerp(orientation, delta, orientation, orientation*rotation, false);

		if(enablelog) {
			outfile << time << "\t" << position << "\t" << velocity << "\t" << orientation << "\t" << rotation << endl;
		}
		time += delta;
	}

	~PhysicsObject() {
		disableLog();
	}
};

#endif
