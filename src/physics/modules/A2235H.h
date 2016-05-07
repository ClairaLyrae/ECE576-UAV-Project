#ifndef GPS_H_
#define GPS_H_

#include <systemc.h>
#include "../Physics.h"
#include <stdint.h>
#include "../../uavcan/UAVCAN.h"
#include "../../uavcan/FrameTypes.h"
#include "../../util/util.h"

#define GPS_LATITUDE 32.2319
#define GPS_LONGITUDE -110.9501

#define M_PI 3.14159265359
#define M_PI_2 1.57079632679
#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.01745329251

class A2235H : public PhysicsComponent, public sc_module
{
private:
	unsigned registers[128];
public:
	static const unsigned char CAN_NODE = 3;
	static const unsigned char CAN_PRIORITY = 10;
	float longitude, latitude, heading, altitude, velocity;
	float period_ms;

	sc_port<uav_can_if> canif;

	SC_HAS_PROCESS(A2235H);

	A2235H(sc_module_name name, double broadcastRate) : sc_module(name) {
		SC_THREAD(main);
		latitude = GPS_LATITUDE;
		longitude = GPS_LONGITUDE;
		period_ms = 1000.0/broadcastRate;
	}

	void main() {
		unsigned char broadcastCount;
		uav_can_msg msg;

		// Continuous broadcasting at fixed rate
		while(true) {
			wait(period_ms,SC_MS);

			// Broadcast latitiude
			while(!canif->can_transmit(CAN_PRIORITY))
				canif->can_listen(msg);
			msg.set(20002, broadcastCount, CAN_NODE, 0);
			msg.packFloat32(latitude);
			cout << "[" << sc_time_stamp() << "] GPS broadcast latitude: " << latitude << endl;
			canif->can_message(msg);

			// Broadcast longitude
			while(!canif->can_transmit(CAN_PRIORITY))
				canif->can_listen(msg);
			msg.set(20003, broadcastCount, CAN_NODE, 0);
			msg.packFloat32(longitude);
			cout << "[" << sc_time_stamp() << "] GPS broadcast longitude: " << longitude << endl;
			canif->can_message(msg);

			// Broadcast heading, velocity, altitude
			while(!canif->can_transmit(CAN_PRIORITY))
				canif->can_listen(msg);
			msg.set(20004, broadcastCount, CAN_NODE, 0);
			msg.packFloat16(heading, velocity, altitude);
			cout << "[" << sc_time_stamp() << "] GPS broadcast HVA" << endl;
			canif->can_message(msg);

			broadcastCount++;
		}
	}

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Update sensor data from physics object
		latitude = GPS_LATITUDE + (parent.position[XAXIS]/111111.0);
		longitude = GPS_LONGITUDE + (parent.position[YAXIS]/(111111.0*cos(latitude*DEG_TO_RAD)));
		velocity = length(Vec2d(parent.velocity[0], parent.velocity[1]));
		heading = parent.attitude[2];
		altitude = parent.position[2];
	}
};

#endif
