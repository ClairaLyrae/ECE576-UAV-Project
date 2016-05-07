#ifndef PROCESSOR_H_
#define PROCESSOR_H_

#define M_PI 3.14159265359

#include <systemc.h>
#include <stdint.h>
#include "UAVCAN.h"
#include "util/util.h"

// Processor Module
class Processor : public sc_module
{
private:
	unsigned i;
	uint8_t message[7];
	unsigned char target, length, transfer, source;
	unsigned short msgType;
	unsigned char lat_c, long_c, gps_c;
	uint8_t hw_req[7];
	unsigned short tf_wait;
	float lati, longi, altt, velo, headn;
	uint32_t temp;
	uint16_t half;

public:
	static const unsigned char CAN_NODE = 1;

	SC_HAS_PROCESS(Processor);

	sc_port<uav_can_if> canif;

	sc_event canFree;

	Processor(sc_module_name name) : sc_module(name) {
		//SC_THREAD(main);
		//SC_THREAD(can_monitor);
	}

	void main() {
		//send setup (configure things here)
		while(!canif->can_transmit(1)) //set priority
		{
			wait(canFree);
		}
		message[0] = 0;
		half = floatToHalf(0);
		message[1] = half & 0x0F;
		message[2] = (half >> 8) & 0x0F;
		half = floatToHalf(0);
		message[3] = half & 0x0F;
		message[4] = (half >> 8) & 0x0F;
		half = floatToHalf(15);
		message[5] = half & 0x0F;
		message[6] = (half >> 8) & 0x0F;
		cout << (int)message[0] << " " << (int)message[5] << " " << (int)message[6] << endl;
		canif->can_message(20001, message, 7, 0, CAN_NODE);

		while(!canif->can_transmit(1)) //set priority
		{
			wait(canFree);
		}
		message[0] = 35;
		half = floatToHalf(0);
		message[1] = half & 0x0F;
		message[2] = (half >> 8) & 0x0F;
		half = floatToHalf(0);
		message[3] = half & 0x0F;
		message[4] = (half >> 8) & 0x0F;
		half = floatToHalf(45*M_PI/180);
		message[5] = half & 0x0F;
		message[6] = (half >> 8) & 0x0F;
		cout << (int)message[0] << " " << (int)message[5] << " " << (int)message[6] << endl;
		canif->can_message(20001, message, 7, 1, CAN_NODE);

		cout << "[" << sc_time_stamp() << "] Targeting altitude of 15m, yaw rate of 5deg/s" << endl;
		wait(5000, SC_MS);

		while(!canif->can_transmit(1)) //set priority
		{
			wait(canFree);
		}
		message[0] = 3;
		half = floatToHalf(0);
		message[1] = half & 0x0F;
		message[2] = (half >> 8) & 0x0F;
		half = floatToHalf(0);
		message[3] = half & 0x0F;
		message[4] = (half >> 8) & 0x0F;
		half = floatToHalf(4);
		message[5] = half & 0x0F;
		message[6] = (half >> 8) & 0x0F;
		cout << (int)message[0] << " " << (int)message[5] << " " << (int)message[6] << endl;
		canif->can_message(20001, message, 7, 2, CAN_NODE);
		cout << "[" << sc_time_stamp() << "] Dropping altitude to 4m" << endl;

		return;
	}

	void can_monitor()
	{
		while(1)
		{
			target = canif->can_listen(msgType, message, length, transfer, source);
			canFree.notify(SC_ZERO_TIME);
			if (target == 0)
			{
				switch(msgType)
				{
				case 20002:
					//lat
					temp = (message[3] << 24) & (message[2] << 16) & (message[1] << 8) & message[0];
					lati = *((float*)&temp);
					lat_c = transfer;
					break;
				case 20003:
					//long
					temp = (message[3] << 24) & (message[2] << 16) & (message[1] << 8) & message[0];
					longi = *((float*)&temp);
					long_c = transfer;
					break;
				case 20004:
					//alt and more
					headn = halfToFloat((uint16_t)((message[1] << 8) & message[0]));
					velo = halfToFloat((uint16_t)((message[3] << 8) & message[2]));
					altt = halfToFloat((uint16_t)((message[5] << 8) & message[4]));
					gps_c = transfer;
					if(lat_c == long_c and long_c == gps_c) {
						//notify gps update
					}
					break;
				case 20005:
					//hw interrupt notify
					break;
				default:
					break;
				}
			} else if (target == CAN_NODE) {
				if(tf_wait == msgType)
				{
					tf_wait = 0;
					for(i = 0; i < length; i++)
					{
						hw_req[i] = message[i];
					}
					//notify response recieved
				}
			}
		}
	}

};

#endif
