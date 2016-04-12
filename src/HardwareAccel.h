#ifndef HARDWARE_ACCEL_H_
#define HARDWARE_ACCEL_H_

#include <systemc.h>

// Hardware Module
class HardwareAccel : public sc_module
{
public:
	SC_HAS_PROCESS(HardwareAccel);

	HardwareAccel(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
	}

	void main() {
		return;
	}
};

#endif
