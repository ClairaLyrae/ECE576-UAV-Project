#include <systemc.h>
#include <ctime>

#include "processor.h"

//////////////////////////////////////////////
// Top Module
//////////////////////////////////////////////

class top : public sc_module
{
public:
	processor* proc;

	SC_CTOR(top) {
		// Processor Module
		proc = new processor("PROC");
	}
};

//////////////////////////////////////////////
// System C Main
//////////////////////////////////////////////

int sc_main(int argc, char *argv[]) {

	// Instantiate modules
	top top_inst("TOP");

	// Start simulation
	cout << "Beginning simulation...\n" << endl;
	sc_start();
	cout << "\nSimulation stopped." << endl;

	return 0;
}
