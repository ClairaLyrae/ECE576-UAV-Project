#ifndef PROCESSOR_H_
#define PROCESSOR_H_

#define M_PI 3.14159265359
#define M_PI_2 1.57079632679
#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.01745329251

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
	bool pid_test_verbose;
	unsigned pid_test;

	bool use_program;
	string program_file;
	ifstream progfile;

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
		pid_test_verbose = false;
		use_program = false;
	}

	bool enableProgram(string filename) {
		program_file = filename;
		progfile.open(filename.c_str());
		if(!progfile.good()) {
			cout << "Could not load program for flight controller" << endl;
			use_program = false;
		}
		use_program = true;
		return use_program;
	}

	void disableProgram() {
		if(use_program)
			progfile.close();
		use_program = false;
	}

	void showCommands(bool b) {
		pid_test_verbose = b;
	}

	void setFlightTest(unsigned type, float period) {
		pid_test = type;
		pid_test_period_ms = period*1000;
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
		if(use_program)
			testProgram();
		else
			testPID();
	}

	void testPID() {
		uav_can_msg msg;
		unsigned i;
		double tgt, tgt2;

		for(i = 0; i < 4; i++) {
			switch(pid_test) {
				case 0:
					return;
				case 1:
					tgt = round(((float)rand()/RAND_MAX)*30) - 10;
					if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor sent altitude command of " << tgt << " m" << endl;
					sendFlightCommand(0xC0, 0.0, 0.0, tgt);
					break;
				case 2:
					tgt = ((float)rand()/RAND_MAX)*6.0 - 2;
					if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor sent vertical velocity command of " << tgt << " m/s" << endl;
					sendFlightCommand(0x81, 0.0, 0.0, tgt);
					break;
				case 3:
					tgt = ((float)rand()/RAND_MAX)*M_PI;
					if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor sent yaw command of " << tgt*180/M_PI << " deg" << endl;
					sendFlightCommand(0xC2, 0.0, 0.0, tgt);
					break;
				case 4:
					tgt = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor sent pitch command of " << tgt*180/M_PI << " deg" << endl;
					sendFlightCommand(0xC2, 0.0, tgt, 0.0);
					break;
				case 5:
					tgt = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor sent roll command of " << tgt*180/M_PI << " deg" << endl;
					sendFlightCommand(0xC2, tgt, 0.0, 0.0);
					break;
				case 6:
					tgt = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					tgt2 = ((float)rand()/RAND_MAX)*(M_PI/2) - (M_PI/4);
					if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor sent roll/pitch command of " << tgt*180/M_PI << " deg, " << tgt2*180/M_PI << " deg"<< endl;
					sendFlightCommand(0xC2, tgt, tgt2, 0.0);
					break;
			}
			wait(pid_test_period_ms, SC_MS);
		}
		return;
	}

	void testProgram() {
		unsigned i, type;
		uint8_t flags;
		float values[3], wait_time;
		string temp, str, str_cmd, str_val;
		bool hasValues, inDeg;
		char c;

		while(!progfile.eof()) {
			getline(progfile, str);
			c = str.at(0);
			if(c == '#' || c == '\n' || c == '\r' || c == ' ')
				continue;
			stringstream ss(str);
			ss >> str_cmd >> str_val;
			if(strcasecmp(str_cmd.c_str(), "wait") == 0) {
				hasValues = false;
				wait_time = atof(str_val.c_str());
				wait(wait_time, SC_MS);
				if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor waiting " << wait_time << "ms" << endl;
			} else if(strcasecmp(str_cmd.c_str(), "pos") == 0) {
				hasValues = true;
				inDeg = false;
				flags = (flags & 0xF0) | 0x00;
			} else if(strcasecmp(str_cmd.c_str(), "vel") == 0) {
				hasValues = true;
				inDeg = false;
				flags = (flags & 0xF0) | 0x01;
			} else if(strcasecmp(str_cmd.c_str(), "att") == 0) {
				hasValues = true;
				inDeg = true;
				flags = (flags & 0xF0) | 0x02;
			} else if(strcasecmp(str_cmd.c_str(), "rate") == 0) {
				hasValues = true;
				inDeg = true;
				flags = (flags & 0xF0) | 0x03;
			} else if(strcasecmp(str_cmd.c_str(), "mode") == 0) {
				hasValues = false;
				flags = 0x00;
				stringstream ss(str_val);
				for(i = 0; i < 4; i++) {
					getline(ss, temp, ',');
					flags |= (atoi(temp.c_str()) == 0 ? 0 : 1) << (7 - i);
				}
				flags &= 0xF0;
				if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor set flags to " << (unsigned)flags << endl;
			}
			if(hasValues) {
				stringstream ss(str_val);
				for(i = 0; i < 3; i++) {
					getline(ss, temp, ',');
					values[i] = atof(temp.c_str());
					if(inDeg)
						values[i] *= DEG_TO_RAD;
				}
				sendFlightCommand(flags, values[0], values[1], values[2]);
				if(pid_test_verbose) cout << "[" << sc_time_stamp() << "] Processor sent command (" << (unsigned)flags << ") of " << values[0] << ", " << values[1] << ", " << values[2] << endl;
			}
		}
		progfile.close();
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
						// cout << "[" << sc_time_stamp() << "] Processor received GPS Latitude" << endl;
						msg.unpackFloat32(gps_latitude);
						break;
					case UAVCAN_GPS_LONG:
						// cout << "[" << sc_time_stamp() << "] Processor received GPS Longitude" << endl;
						msg.unpackFloat32(gps_longitude);
						break;
					case UAVCAN_GPS_HVA:
						// cout << "[" << sc_time_stamp() << "] Processor received GPS HVA" << endl;
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
