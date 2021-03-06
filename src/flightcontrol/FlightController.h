#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include <systemc.h>
#include "../PWM.h"
#include "../I2C.h"
#include "../uavcan/UAVCAN.h"
#include "../uavcan/FrameTypes.h"
#include "../uavcan/FrameGenerator.h"
#include "AltitudeEstimator.h"
#include "PID.h"
#include "Commands.h"
#include "../physics/PhysicsModules.h"
#include "SensorData.h"
#include "gmtl/gmtl.h"
#include <stdint.h>
#include "../util/util.h"

#define I2C_WRITE false
#define I2C_READ true

#define M_PI 3.14159265359
#define M_PI_2 1.57079632679
#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.01745329251

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#define ROLL 0
#define PITCH 1
#define YAW 2

#define MIN_THROTTLE 0
#define MAX_THROTTLE 1

#define ACCEL_SCALE_BIAS 0
#define GYRO_SCALE_BIAS 0
#define MAG_SCALE_BIAS 0
#define TEMP_SCALE_BIAS 0
#define PRESSURE_SCALE_BIAS 0

#define PITCH_LIMIT 0.349065
#define ROLL_LIMIT 0.349065
#define PITCH_COMPENSATION_GAIN 1
#define ROLL_COMPENSATION_GAIN 1


#define AIR_MOLAR_MASS 0.0289644	// kg/mol
#define TEMP_LAPSE_RATE 0.0065		// K/m
#define GAS_CONSTANT 8.31447		// J/(mol*K)
#define EARTH_GRAVITY 9.80665		// m/s^2
#define AIR_DENSITY 1.225			//
#define STP_TEMP 288.15				// K
#define STP_PRESSURE 101325			// Pa

#define ACC_GYRO_UPDATE_RATE 200
#define MAG_UPDATE_RATE 200
#define BARO_UPDATE_RATE 200

using namespace gmtl;

// Hardware Module
class FlightController : public sc_module
{
private:
	ofstream outfile;
	string outfilename;
	bool enablelog;

	// Interrupt System
	sc_event timerInterrupt;
	bool I2C_mag_update;
	bool I2C_acc_gyro_update;
	bool I2C_baro_update;
	int I2C_mag_interval_ms;
	int I2C_acc_gyro_interval_ms;
	int I2C_baro_interval_ms;

	double AHRS_interval_ms;

	bool UAVCAN_report_pos;
	bool UAVCAN_report_vel;
	bool UAVCAN_report_att;
	bool UAVCAN_report_rate;
	double UAVCAN_interval_ms;

	PhysicsSim *sim;
	PhysicsObject *uav;
public:
	static unsigned CAN_NODE;
	static unsigned CAN_PRIORITY;

	// Ports
	sc_port<i2c_mst_if> i2c_mst;
	sc_port<PWM_out> pwm_out_1;
	sc_port<PWM_out> pwm_out_2;
	sc_port<PWM_out> pwm_out_3;
	sc_port<PWM_out> pwm_out_4;
	sc_port<uav_can_if> canif;

	// Motor output values
	double motor[4];


	// Sensor/Command data
	Sensors sensor;
	Command cmd;

	// Calculated IMU Data
	Vec3d attitude_rate;
	Vec3d attitude;
	Vec3d acc;
	Vec3d pos;
	Vec3d vel;
	double baro_altitude;
	double altitude;

	// PIDs
	bool enablePID;
	PID attPID[3];
	PID ratePID[3];
	PID posPID[3];
	PID velPID[3];

	SC_HAS_PROCESS(FlightController);

	FlightController(sc_module_name name, PhysicsSim* physim, PhysicsObject* vehicle) : sc_module(name) {
		SC_THREAD(mainI2C);
		SC_THREAD(mainIMU);
		SC_THREAD(mainTimer);
		SC_THREAD(mainPWM);
		SC_THREAD(mainUAVCAN);
		SC_THREAD(mainUAVCANOut);

		this->sim = physim;
		this->uav = vehicle;

		// Initialize PIDs
		posPID[XAXIS].initialize(20, 20, 0, 20, 0, 0);
		posPID[YAXIS].initialize(20, 20, 0, 20, 0, 0);
		posPID[ZAXIS].initialize(20, 20, 0, 20, 0, 0);
		velPID[XAXIS].initialize(20, 20, 0, 20, 0, 0);
		velPID[YAXIS].initialize(20, 20, 0, 20, 0, 0);
		velPID[ZAXIS].initialize(20, 20, 0, 20, 0, 0);
		attPID[ROLL].initialize(2, 2, 0, M_PI_2, 0, 0);
		attPID[PITCH].initialize(2, 2, 0, M_PI_2, 0, 0);
		attPID[YAW].initialize(2, 2, 0, M_PI_2, 0, 0);
		ratePID[ROLL].initialize(7, 1, 0, 1, 0, 0);
		ratePID[PITCH].initialize(7, 1, 0, 1, 0, 0);
		ratePID[YAW].initialize(12, 1, 0, 1, 0, 0);
		enablePID = true;
		enablelog = false;
		cmd.posCmd[XAXIS] = 0;	cmd.posCmd[YAXIS] = 0;	cmd.posCmd[ZAXIS] = 0;
		cmd.attCmd[ROLL] = 0;	cmd.attCmd[PITCH] = 0;	cmd.attCmd[YAW] = 0;
		cmd.rateCmd[ROLL] = 0;	cmd.rateCmd[PITCH] = 0;	cmd.rateCmd[YAW] = 0;
		AHRS_interval_ms = 1;
		I2C_baro_interval_ms = 1000;
		I2C_acc_gyro_interval_ms = 10;
		I2C_mag_interval_ms = 100;
		UAVCAN_report_pos = false;
		UAVCAN_report_vel = false;
		UAVCAN_report_att = false;
		UAVCAN_report_rate = false;
		UAVCAN_interval_ms = 100;
	}

	void mainPWM();
	void mainIMU();
	void mainPilot();
	void mainI2C();
	void mainTimer();
	void mainUAVCAN();
	void mainUAVCANOut();

	void setMagRate(double rate) {
		I2C_mag_interval_ms = (int)(10000.0/rate);
	}
	void setAccGyroRate(double rate) {
		I2C_acc_gyro_interval_ms = (int)(10000.0/rate);
	}
	void setBaroRate(double rate) {
		I2C_baro_interval_ms = (int)(10000.0/rate);
	}

	void setThrottle(double m1, double m2, double m3, double m4) {
		motor[0] = m1;
		motor[1] = m2;
		motor[2] = m3;
		motor[3] = m4;
	}

	void enableAHRSReports(bool p, bool v, bool a, bool r) {
		UAVCAN_report_pos = p;
		UAVCAN_report_vel = v;
		UAVCAN_report_att = a;
		UAVCAN_report_rate = r;
	}

	void setAHRSReportRate(double freq) {
		UAVCAN_interval_ms = 1000.0/freq;
	}

	void writeAllMotors() {
		pwm_out_1->writePWM(motor[0]);
		pwm_out_2->writePWM(motor[1]);
		pwm_out_3->writePWM(motor[2]);
		pwm_out_4->writePWM(motor[3]);
	}

	bool enableLog(string ofile) {
		this->outfilename = ofile;
    	outfile.open(outfilename.c_str());
		enablelog = true;
		if(!outfile.good()) {
			enablelog = false;
		}
		return enablelog;
	}

	void disableLog() {
		if(enablelog) {
			outfile.flush();
			outfile.close();
		}
		enablelog = false;
	}

	float wrapRadians(float angle)
	{
	    if (angle >= M_PI)
	        return (angle - 2*M_PI);
	    else if (angle < -M_PI)
	        return (angle + 2*M_PI);
	    else
	        return (angle);
	}

	double constrain(double input, double minValue, double maxValue)
	{
	    if (input < minValue)
	        return minValue;
	    else if (input > maxValue)
	        return maxValue;
	    else
	        return input;
	}

	void sendFlightReport(uint8_t header, float x, float y, float z) {
		uav_can_msg msg;
		while(!canif->can_transmit(CAN_PRIORITY))
			canif->can_listen(msg);
		msg.set(UAVCAN_FLIGHT_REP, 0, CAN_NODE, 0);
		msg.packByteFloat16(header, x, y, z);
		canif->can_message(msg);
	}

	~FlightController() {
		disableLog();
	}
};

/////////////////////////////////////////////////
// System C Threads
/////////////////////////////////////////////////

// Updates motor PWM output
void FlightController::mainPWM() {
	while(true) {
		writeAllMotors();
		wait(1, SC_MS);
	}
}

// Internal timer, generates interrupts at specific intervals
// Also updates the motor PWM outputs
void FlightController::mainTimer() {
	unsigned updateCycle;
	I2C_mag_update = false;
	I2C_acc_gyro_update = false;
	I2C_baro_update = false;
	while(true) {
		updateCycle++;
		if(updateCycle >= 100000)	// Roll over every 10s
			updateCycle = 0;
		if((updateCycle%I2C_baro_interval_ms == 0)) {
			if(I2C_mag_update)
				cout << "[" << sc_time_stamp() << "] I2C Bus error: unserviced baro request" << endl;
			I2C_baro_update = true;
		}
		if((updateCycle%I2C_acc_gyro_interval_ms == 0)) {
			if(I2C_mag_update)
				cout << "[" << sc_time_stamp() << "] I2C Bus error: unserviced acc/gyro request" << endl;
			I2C_acc_gyro_update = true;
			}
		if((updateCycle%I2C_mag_interval_ms == 0)) {
			if(I2C_mag_update)
				cout << "[" << sc_time_stamp() << "] I2C Bus error: unserviced mag request" << endl;
			I2C_mag_update = true;
		}
		if(I2C_mag_update || I2C_acc_gyro_update || I2C_baro_update)
			timerInterrupt.notify();
		wait(0.1, SC_MS);	// 100us interval
	}
}

// Implements I2C communication with DMA, places results into raw sensor value registers
// Does hardware averaging in DMA location, summing results and keeping track of
// result count.
// Uses ISF model with interrupt flags.
void FlightController::mainI2C() {
	unsigned i;
	uint8_t buffer[16];
	uint32_t raw;
	while(true) {
		wait(timerInterrupt);

		// Read Accel/Gyro
		if(I2C_acc_gyro_update) {
			i2c_mst->i2c_read(LSM6DS3::I2C_ADDRESS, LSM6DS3::REG_GYRO_X_L, 12, buffer);
			for (i = 0; i < 3; i++) {
				raw = ((buffer[2*i + 1] << 8) + buffer[2*i]);
				sensor.gyro[i] = (((int16_t)raw)/(LSM6DS3::GYRO_SCALE_FACTOR) + GYRO_SCALE_BIAS)*(M_PI/180);
				raw = ((buffer[6 + (2*i + 1)] << 8) + buffer[6 + 2*i]);
				sensor.accel[i] = ((int16_t)raw)/(LSM6DS3::ACC_SCALE_FACTOR) + ACCEL_SCALE_BIAS;
			}
			I2C_acc_gyro_update = false;
		}

		// Read mag
		if(I2C_mag_update) {
			i2c_mst->i2c_read(LIS3MDL::I2C_ADDRESS, LIS3MDL::REG_MAG_X_L, 6, buffer);
			for (i = 0; i < 3; i++) {
				raw = (buffer[2*i + 1] << 8) + buffer[2*i];
				sensor.mag[i] = raw/(LIS3MDL::MAG_SCALE_FACTOR) + MAG_SCALE_BIAS;
			}
			I2C_mag_update = false;
		}

		// Read temperature and pressure from barometer (100Hz), use first order filtering
		if(I2C_baro_update) {
			i2c_mst->i2c_read(LPS22HB::I2C_ADDRESS, LPS22HB::REG_PRES_XL, 5, buffer);
			raw = (buffer[2] << 16) + (buffer[1] << 8) + buffer[0];
			sensor.pressure = raw/(LPS22HB::PRESS_SCALE_FACTOR) + PRESSURE_SCALE_BIAS;
			raw = (buffer[4] << 8) + buffer[3];
			sensor.temp = raw/(LPS22HB::TEMP_SCALE_FACTOR) + TEMP_SCALE_BIAS;
			baro_altitude = (STP_TEMP/TEMP_LAPSE_RATE) * (1.0f - pow(sensor.pressure / STP_PRESSURE, (GAS_CONSTANT*TEMP_LAPSE_RATE)/(EARTH_GRAVITY*AIR_MOLAR_MASS)));
			I2C_baro_update = false;
		}
	}
}

// IMU/PID control thread. Does AHRS calculation to get attitude solution and applies
// the PID controller to the motor outputs if enabled
void FlightController::mainIMU() {
	double error;
	double throttleCmd;
	double tempAttCompensationPitch, tempAttCompensationRoll;
	unsigned i;
	double maxMotor;
	double dt;

	while(true) {
		dt = 0.001*AHRS_interval_ms; // Interval (s)

		// Read sensors
		//attitude_rate = uav->attitude_rate;
		//acc = uav->acceleration;
		attitude_rate.set(sensor.gyro[ROLL], sensor.gyro[PITCH], sensor.gyro[YAW]);
		acc.set(sensor.accel[XAXIS], sensor.accel[YAXIS], sensor.accel[ZAXIS]);

		// AHRS
		attitude[ROLL] += wrapRadians(attitude_rate[ROLL]*dt);
		attitude[PITCH] += wrapRadians(attitude_rate[PITCH]*dt);
		attitude[YAW] += wrapRadians(attitude_rate[YAW]*dt);
		vel += acc*dt;
		pos += vel*dt;

		// Apply PID Controllers
		if(enablePID) {

			// Compute error term for position based off estimation/integration, apply to PIDs as feedback
			if(cmd.enableLatPID) {
				error = cmd.posCmd[XAXIS] - pos[XAXIS];
				cmd.velCmd[XAXIS] = posPID[XAXIS].update(error, dt);
//				error = cmd.posCmd[YAXIS] - pos[YAXIS];
//				cmd.velCmd[YAXIS] = posPID[YAXIS].update(error, dt);

				error = cmd.velCmd[XAXIS] - vel[XAXIS];
				cmd.attCmd[PITCH] = -velPID[XAXIS].update(error, dt);
//				error = cmd.velCmd[YAXIS] - vel[YAXIS];
//				cmd.attCmd[PITCH] = -velPID[YAXIS].update(error, dt);

				constrain(cmd.attCmd[PITCH], -M_PI_4, M_PI_4);
				constrain(cmd.attCmd[ROLL], -M_PI_4, M_PI_4);
			}

			// Compute error terms for roll and pitch in attitude mode, apply to PIDs as feedback
			if(cmd.enableAttPID) {
				error = wrapRadians(cmd.attCmd[ROLL] - attitude[ROLL]);
				cmd.rateCmd[ROLL] = attPID[ROLL].update(error, dt);
				error = wrapRadians(cmd.attCmd[PITCH] + attitude[PITCH]);
				cmd.rateCmd[PITCH] = attPID[PITCH].update(error, dt);
				error = wrapRadians(cmd.attCmd[YAW] - attitude[YAW]);
				cmd.rateCmd[YAW] = attPID[YAW].update(error, dt);
			}

			// Compute error terms for roll, pitch and yaw from rate, apply to PIDs as feedback
			error = cmd.rateCmd[ROLL] - attitude_rate[ROLL];
			ratePID[ROLL].update(error, dt);
			error = cmd.rateCmd[PITCH] + attitude_rate[PITCH];
			ratePID[PITCH].update(error, dt);
			error = cmd.rateCmd[YAW] - attitude_rate[YAW];
			ratePID[YAW].update(error, dt);

			// Compute error term for position based off estimation/integration, apply to PIDs as feedback
			if(cmd.enableAltPID) {
				error = cmd.posCmd[ZAXIS] - pos[ZAXIS];
				cmd.velCmd[ZAXIS] = posPID[ZAXIS].update(error, dt);
			}

			// Compute error term for velocity based off estimation/integration, apply to PIDs as feedback
			error = cmd.velCmd[ZAXIS] - vel[ZAXIS];
			throttleCmd = 0.5 + velPID[ZAXIS].update(error, dt);

			// Compute Cosine of constrained Roll/Pitch Angles and Multiply by Att-Alt Gain
			tempAttCompensationRoll = constrain(attitude[ROLL], -ROLL_LIMIT,  ROLL_LIMIT);
			tempAttCompensationRoll = ROLL_COMPENSATION_GAIN / cosf(tempAttCompensationRoll);
			tempAttCompensationPitch = constrain(attitude[PITCH], -PITCH_LIMIT,  PITCH_LIMIT);
			tempAttCompensationPitch = PITCH_COMPENSATION_GAIN / cosf(tempAttCompensationPitch);

			// Apply Att Compensations to Throttle Command
			throttleCmd = throttleCmd*tempAttCompensationRoll*tempAttCompensationPitch;

			// Motor Mixing (assign motor outputs to achieve desired attitude change)
			motor[0] = throttleCmd;     // Front left (CW)
			motor[1] = throttleCmd;     // Front right (CCW)
			motor[2] = throttleCmd;     // Back right (CW)
			motor[3] = throttleCmd;     // Back left  (CCW)

			motor[0] += ratePID[ROLL].state;
			motor[1] += -ratePID[ROLL].state;
			motor[2] += -ratePID[ROLL].state;
			motor[3] += ratePID[ROLL].state;
			motor[0] += -ratePID[PITCH].state;
			motor[1] += -ratePID[PITCH].state;
			motor[2] += ratePID[PITCH].state;
			motor[3] += ratePID[PITCH].state;
			motor[0] += -ratePID[YAW].state;
			motor[1] += ratePID[YAW].state;
			motor[2] += -ratePID[YAW].state;
			motor[3] += ratePID[YAW].state;

		    maxMotor = motor[0];
		    for (i = 0; i < 4; i++) {
		        if (motor[i] > maxMotor)
		            maxMotor = motor[i];
		    }
		    // If a motor is run beyond its max speed, scale motors to it
	        if (maxMotor > 1) {
				for (i = 0; i < 4; i++) {
					motor[i] = motor[i]/maxMotor;
				}
	        }

			// Motor log file output
			if(enablelog) {
				outfile << sc_time_stamp() << motor[0] << "\t" << motor [1] << "\t" << motor [2] << "\t" << motor [3] << endl;
			}
		}
		wait(AHRS_interval_ms, SC_MS);
	}
}

void FlightController::mainUAVCAN() {
	uav_can_msg msg;

	while(true)
	{
		canif->can_listen(msg);
		if (msg.isBroadcast()) {
			switch(msg.msgID) {
				case UAVCAN_GPS_LAT:
					msg.unpackFloat32(sensor.gps_latitude);
					// cout << "[" << sc_time_stamp() << "] Flight controller received GPS Latitude" << endl;
					break;
				case UAVCAN_GPS_LONG:
					msg.unpackFloat32(sensor.gps_longitude);
					// cout << "[" << sc_time_stamp() << "] Flight controller received GPS Longitude" << endl;
					break;
				case UAVCAN_GPS_HVA:
					msg.unpackFloat16(sensor.gps_heading, sensor.gps_velocity, sensor.gps_altitude);
					// cout << "[" << sc_time_stamp() << "] Flight controller received GPS HVA" << endl;
					break;
				default:
					break;
			}
		} else if(msg.destID == CAN_NODE && msg.msgID == UAVCAN_FLIGHT_CON) {
			float fa, fb, fc;
			uint8_t header, dataType;

			msg.unpackByteFloat16(header, fa, fb, fc);
			cmd.enableAttPID = ((header & 0x80) != 0);
			cmd.enableAltPID = ((header & 0x40) != 0);
			cmd.enableLatPID = ((header & 0x20) != 0);
			dataType = header & 0x0F;
			switch(dataType)
			{
			case 0:
				cmd.posCmd[XAXIS] = fa;
				cmd.posCmd[YAXIS] = fb;
				cmd.posCmd[ZAXIS] = fc;
				cout << "[" << sc_time_stamp() << "] Flight controller received pos cmd: " << fa << " m, " << fb << " m, " << fc << " m" << endl;
				break;
			case 1:	// Velocity command
				cmd.velCmd[XAXIS] = fa;
				cmd.velCmd[YAXIS] = fb;
				cmd.velCmd[ZAXIS] = fc;
				cout << "[" << sc_time_stamp() << "] Flight controller received vel cmd: " << fa << " m/s, " << fb << " m/s, " << fc << " m/s" << endl;
				break;
			case 2:
				cmd.attCmd[ROLL] = fa;
				cmd.attCmd[PITCH] = fb;
				cmd.attCmd[YAW] = fc;
				cout << "[" << sc_time_stamp() << "] Flight controller received att cmd: " << fa*RAD_TO_DEG << " deg, " << fb*RAD_TO_DEG << " deg, " << fc*RAD_TO_DEG << " deg" << endl;
				break;
			case 3:
				cmd.rateCmd[ROLL] = fa;
				cmd.rateCmd[PITCH] = fb;
				cmd.rateCmd[YAW] = fc;
				cout << "[" << sc_time_stamp() << "] Flight controller received rate cmd: " << fa*RAD_TO_DEG << " deg/s, " << fb*RAD_TO_DEG << " deg/s, " << fc*RAD_TO_DEG << " deg/s" << endl;
				break;
			default:
				cout << "[" << sc_time_stamp() << "] Flight controller failed to interpret command." << endl;
				break;
			}
		}
	}
}


void FlightController::mainUAVCANOut() {
	while(true) {
		if(UAVCAN_report_pos) {
			cout << "[" << sc_time_stamp() << "] Flight controller reported pos: " << pos[XAXIS] << " m, " << pos[YAXIS] << " m, " << pos[ZAXIS] << " m" << endl;
			sendFlightReport(0x00, (float)pos[XAXIS], (float)pos[YAXIS], (float)pos[ZAXIS]);
		}
		if(UAVCAN_report_vel) {
			cout << "[" << sc_time_stamp() << "] Flight controller reported vel: " << pos[XAXIS] << " m/s, " << pos[YAXIS] << " m/s, " << pos[ZAXIS] << " m/s" << endl;
			sendFlightReport(0x01, (float)vel[XAXIS], (float)vel[YAXIS], (float)vel[ZAXIS]);
		}
		if(UAVCAN_report_att) {
			cout << "[" << sc_time_stamp() << "] Flight controller reported att: " << attitude[ROLL] << " deg, " << attitude[PITCH] << " deg, " << attitude[YAW] << " deg" << endl;
			sendFlightReport(0x02, (float)attitude[XAXIS], (float)attitude[YAXIS], (float)attitude[ZAXIS]);
		}
		if(UAVCAN_report_rate) {
			cout << "[" << sc_time_stamp() << "] Flight controller reported rate: " << attitude_rate[ROLL] << " deg/s, " << attitude_rate[PITCH] << " deg/s, " << attitude_rate[YAW] << " deg/s" << endl;
			sendFlightReport(0x03, (float)attitude_rate[XAXIS], (float)attitude_rate[YAXIS], (float)attitude_rate[ZAXIS]);
		}
		wait(UAVCAN_interval_ms, SC_MS);
	}
}

unsigned FlightController::CAN_NODE = 2;
unsigned FlightController::CAN_PRIORITY = 2;

#endif
