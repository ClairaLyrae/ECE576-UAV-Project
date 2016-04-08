#ifndef SRC_PROCESSOR_H_
#define SRC_PROCESSOR_H_

#include <systemc.h>

//////////////////////////////////////////////
// Processor Module
//////////////////////////////////////////////

class processor : public sc_module
{
public:
	SC_HAS_PROCESS(processor);

	processor(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
	}

	void main() {
		return;
	}
};

#endif
