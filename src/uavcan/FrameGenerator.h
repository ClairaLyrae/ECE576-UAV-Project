#ifndef UAVCAN_FRAMEGENERATOR_H_
#define UAVCAN_FRAMEGENERATOR_H_

#include <systemc.h>
#include "UAVCAN.h"
#include "FrameTypes.h"

class FrameGenerator : public sc_module
{
private:
	double wait_min, wait_var;
	unsigned len_min, len_var;
	bool en;
public:
	static unsigned CAN_PRIORITY, CAN_NODE;

	sc_port<uav_can_if> canif;

	SC_HAS_PROCESS(FrameGenerator);

	FrameGenerator(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
		en = true;
		len_var = 0;
		len_min = 7;
		wait_var = 1000;
		wait_min = 100;
	}

	void setSize(unsigned range, unsigned min) {
		len_var = range;
		len_min = min;
		if(len_min > 7)
			len_min = 7;
		if(len_min + len_var > 7)
			len_var += (7 - len_var + len_min);
	}

	void setRate(double range, double min) {
		wait_var = range;
		wait_min = min;
	}

	void enable(bool b) {
		en = b;
	}

	void main()
	{
		uav_can_msg msg;
		srand(time(0));
		while(en)	{
			wait(((double)rand()/RAND_MAX)*wait_var + wait_min, SC_MS);
			while(!canif->can_transmit(CAN_PRIORITY))
				canif->can_listen(msg);
			msg.set(UAVCAN_TRAFFIC, 0, CAN_NODE, 0);
			msg.packEmpty((rand() % len_var) + len_min);
			canif->can_message(msg);
		}
	}
};

unsigned FrameGenerator::CAN_PRIORITY = 31;
unsigned FrameGenerator::CAN_NODE = 5;

#endif
