#ifndef GPS_H_
#define GPS_H_

#include <systemc.h>
#include "../Physics.h"
#include <stdint.h>
#include "../../UAVCAN.h"
#include "../../util/extrause.h"

class A2235H : public PhysicsComponent, public sc_module
{
private:
	unsigned registers[128];
public:
	static const unsigned char CAN_NODE = 3;
	static const unsigned char COORD_PRI = 10;//change me!
	float longitude, latitude, heading, altitude, velocity;

	sc_port<uav_can_if> canif;

	SC_HAS_PROCESS(A2235H);

	A2235H(sc_module_name name) : sc_module(name) {
		//SC_THREAD(main);
		latitude = 32.2319;
		longitude = -110.9501;
	}

	void main() {
		unsigned char broadcastCount;
		unsigned char transfer, source, length;
		unsigned short msgType;
		uint8_t message[7];
		uint32_t temp;
		uint16_t half;

		while(true) {
			wait(100,SC_MS);
			while(!canif->can_transmit(COORD_PRI))
			{
				canif->can_listen(msgType, message, length, transfer, source);
			}
			// broadcast lat
			temp = *((uint32_t*)&latitude);
			message[0] = temp & 0x0F;
			message[1] = (temp >> 8) & 0x0F;
			message[2] = (temp >> 16) & 0x0F;
			message[3] = (temp >> 24) & 0x0F;
			canif->can_message(20002,message,4,broadcastCount,CAN_NODE);
			while(!canif->can_transmit(COORD_PRI))
			{
				canif->can_listen(msgType, message, length, transfer, source);
			}
			// broadcast long
			temp = *((uint32_t*)&longitude);
			message[0] = temp & 0x0F;
			message[1] = (temp >> 8) & 0x0F;
			message[2] = (temp >> 16) & 0x0F;
			message[3] = (temp >> 24) & 0x0F;
			canif->can_message(20003,message,4,broadcastCount,CAN_NODE);
			while(!canif->can_transmit(COORD_PRI))
			{
				canif->can_listen(msgType, message, length, transfer, source);
			}
			// broadcast rest
			half = floatToHalf(heading);
			message[0] = half & 0x0F;
			message[1] = (half >> 8) & 0x0F;
			half = floatToHalf(velocity);
			message[2] = half & 0x0F;
			message[3] = (half >> 8) & 0x0F;
			half = floatToHalf(altitude);
			message[4] = half & 0x0F;
			message[5] = (half >> 8) & 0x0F;
			canif->can_message(20004,message,6,broadcastCount,CAN_NODE);
			broadcastCount++;
		}
	}

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Update sensor data from physics object
		// TODO Calculate GPS values
		velocity = length(Vec2d(parent.velocity[0], parent.velocity[1]));
		heading = parent.attitude[2]; // Radians
		altitude = parent.position[2];
	}
};

#endif
