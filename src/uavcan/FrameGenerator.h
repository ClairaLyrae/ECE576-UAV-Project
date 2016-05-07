#ifndef UAVCAN_FRAMEGENERATOR_H_
#define UAVCAN_FRAMEGENERATOR_H_

#include <systemc.h>
#include "UAVCAN.h"
#include "FrameTypes.h"

class uavcan_traffic_gen : public sc_module
{
public:
	sc_port<uav_can_if> canif;

	SC_HAS_PROCESS(uavcan_traffic_gen);

	uavcan_traffic_gen(sc_module_name name, unsigned freq_min, unsigned freq_rng, unsigned char priority, unsigned char node) : sc_module(name), wait_min(freq_min), wait_var(freq_rng), CAN_PRIORITY(priority), CAN_NODE(node)
	{
		SC_THREAD(noise);
	}

	void noise()
	{
		uav_can_msg msg;
		srand(time(0));
		while(1)
		{
			wait(rand() % wait_var + wait_min, SC_NS);
			while(!canif->can_transmit(CAN_PRIORITY))
				canif->can_listen(msg);
			msg.set(UAVCAN_TRAFFIC, 0, CAN_NODE, 0);
			canif->can_message(msg);
		}
	}

private:
	unsigned wait_min, wait_var;
	unsigned int CAN_PRIORITY, CAN_NODE;
};


#endif
