#ifndef PHYSICS_SIM_H_
#define PHYSICS_SIM_H_

#include <systemc.h>
#include "gmtl/gmtl.h"

using namespace gmtl;
using namespace std;

// Physics SystemC Thread
class PhysicsSim : public sc_module
{
public:
	std::vector<PhysicsObject*> objects;
	double delta_ms;
	double time;
	double sim_time;

	sc_event event_sim_delta;
	sc_event event_sim_trigger;

	SC_HAS_PROCESS(PhysicsSim);

	PhysicsSim(sc_module_name name, double delta_ms, double sim_time) : sc_module(name) {
		SC_THREAD(main);
		this->delta_ms = delta_ms;
		this->sim_time = sim_time;
		this->time = 0;
	}

	PhysicsSim(sc_module_name name, double delta_ms) : PhysicsSim(name, delta_ms, 0) {}

	void addObject(PhysicsObject* p) {
		objects.push_back(p);
	}

	double timeElapsed() {
		return time;
	}

	void main() {
		while(sim_time == 0 || time < sim_time) {
			wait(delta_ms, SC_MS);
			for(PhysicsObject* p : objects) {
				p->update(delta_ms*0.001);
			}
			time += delta_ms*0.001;
			event_sim_delta.notify();
		}
		return;
	}
};

#endif
