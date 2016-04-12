#ifndef PROCESSOR_H_
#define PROCESSOR_H_

#include <systemc.h>

// Processor Module
class Processor : public sc_module
{
public:
	SC_HAS_PROCESS(Processor);

	Processor(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
	}

	void main() {
		return;
	}
};

#endif
