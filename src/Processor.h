#ifndef PROCESSOR_H_
#define PROCESSOR_H_

#define M_PI 3.14159265359

#include <systemc.h>
#include <stdint.h>
#include "util/util.h"
#include "flightcontrol/FlightController.h"
#include "uavcan/UAVCAN.h"
#include "uavcan/FrameTypes.h"

// Processor Module
class Processor : public sc_module
{
private:
	float gps_latitude, gps_longitude, gps_heading, gps_altitude, gps_velocity;
	float pid_test_period_ms;
	unsigned pid_test;

public:
	static const unsigned char CAN_NODE = 1;
	static const unsigned char CAN_PRIORITY = 5;

	SC_HAS_PROCESS(Processor);

	sc_port<uav_can_if> canif;

	sc_event canFree;

	Processor(sc_module_name name) : sc_module(name) {
		SC_THREAD(mainSW);
		SC_THREAD(mainUAVCAN);
		pid_test = 5;
		pid_test_period_ms = 5000.0;
	}

	void setPIDTest(unsigned type, float period_ms) {
		pid_test = type;
		pid_test_period_ms = period_ms;
	}

	void sendFlightCommand(uint8_t header, float x, float y, float z) {
		uav_can_msg msg;
		while(!canif->can_transmit(CAN_PRIORITY))
			canif->can_listen(msg);
		msg.set(UAVCAN_FLIGHT_CON, 0, CAN_NODE, FlightController::CAN_NODE);
		msg.packByteFloat16(header, x, y, z);
		canif->can_message(msg);
	}

	void mainSW() {
		uav_can_msg msg;

		// Initialization
		unsigned i;
		double tgt, tgt2;
		for(i = 0; i < 4; i++) {
			switch(pid_test) {
				case 0:
					tgt = round(((float)rand()/RAND_MAX)*30) - 10;
					cout << "[" << sc_time_stamp() << "] Processor sent altitude of " << tgt << " m" << endl;
					sendFlightCommand(0xC0, 0.0, 0.0, tgt);
					break;
				case 1:
					tgt = ((float)rand()/RAND_MAX)*6.0 - 2;
					cout << "[" << sc_time_stamp() << "] Processor sent vertical velocity of " << tgt << " m/s" << endl;
					sendFlightCommand(0x81, 0.0, 0.0, tgt);
					break;
				case 2:
					tgt = ((float)rand()/RAND_MAX)*M_PI;
					cout << "[" << sc_time_stamp() << "] Processor sent yaw of " << tgt*180/M_PI << " deg" << endl;
					sendFlightCommand(0xC2, 0.0, 0.0, tgt);
					break;
				case 3:
					tgt = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					cout << "[" << sc_time_stamp() << "] Processor sent pitch of " << tgt*180/M_PI << " deg" << endl;
					sendFlightCommand(0xC2, 0.0, tgt, 0.0);
					break;
				case 4:
					tgt = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					cout << "[" << sc_time_stamp() << "] Processor sent roll of " << tgt*180/M_PI << " deg" << endl;
					sendFlightCommand(0xC2, tgt, 0.0, 0.0);
					break;
				case 5:
					tgt = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					tgt2 = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					cout << "[" << sc_time_stamp() << "] Processor sent roll/pitch of " << tgt*180/M_PI << " deg, " << tgt2*180/M_PI << " deg"<< endl;
					sendFlightCommand(0xC2, tgt, tgt2, 0.0);
					break;
			}
			wait(pid_test_period_ms, SC_MS);
		}
		return;
	}

	void mainUAVCAN()
	{
		uav_can_msg msg;
		while(true) {
			canif->can_listen(msg);
			if (msg.isBroadcast()) {
				switch(msg.msgID) {
					case UAVCAN_GPS_LAT:

						cout << "[" << sc_time_stamp() << "] Processor received GPS Latitude" << endl;
						msg.unpackFloat32(gps_latitude);
						break;
					case UAVCAN_GPS_LONG:
						cout << "[" << sc_time_stamp() << "] Processor received GPS Longitude" << endl;
						msg.unpackFloat32(gps_longitude);
						break;
					case UAVCAN_GPS_HVA:
						cout << "[" << sc_time_stamp() << "] Processor received GPS HVA" << endl;
						msg.unpackFloat16(gps_heading, gps_velocity, gps_altitude);
						break;
					default:
						break;
				}
			}
		}
	}

};

#endif
