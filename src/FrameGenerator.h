#ifndef FRAMEGENERATOR_H_
#define FRAMEGENERATOR_H_

#include <systemc.h>
#include "UAVCAN.h"

class uavcan_traffic_gen : public sc_module
{
public:
	sc_port<uav_can_if> canif;

	SC_HAS_PROCESS(uavcan_traffic_gen);

	uavcan_traffic_gen(sc_module_name name, unsigned freq_min, unsigned freq_rng, unsigned char priority, unsigned char node) : sc_module(name), wait_min(freq_min), wait_var(freq_rng), priority(priority), node(node)
	{
		SC_THREAD(noise);
	}

	void noise()
	{
		srand(time(0));
		while(1)
		{
			wait(rand() % wait_var + wait_min, SC_NS);
			while(!canif->can_transmit(priority))
			{
				target = canif->can_listen(msgType, message, length, transfer, source);
			}
			canif->can_message(20099,traff,7,0,node);
		}
	}

private:
	unsigned wait_min, wait_var;
	unsigned short msgType;
	uint8_t message[7];
	uint8_t traff[7];
	unsigned char transfer, source, target, length, priority, node;
};


#endif
