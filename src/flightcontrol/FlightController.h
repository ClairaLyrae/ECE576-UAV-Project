#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include <systemc.h>
#include "../PWM.h"
#include "../I2C.h"
#include "AltitudeEstimator.h"
#include "PID.h"
#include "AHRS.h"
#include "../physics/PhysicsModules.h"
#include "SensorData.h"
#include "gmtl/gmtl.h"

#define I2C_WRITE false
#define I2C_READ true

#define PID_ROLL
#define PID_PITCH
#define PID_YAW

#define M_PI 3.14159265359
#define M_PI_2 1.57079632679

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
public:
	// Ports
	sc_port<i2c_mst_if> i2c_mst;
	sc_port<PWM_out> pwm_out_1;
	sc_port<PWM_out> pwm_out_2;
	sc_port<PWM_out> pwm_out_3;
	sc_port<PWM_out> pwm_out_4;

	// Motor output values
	double motor[4];

	// Sensor data
	Sensors sensor_raw;
	Sensors sensor;

	// Calculated IMU Data
	Vec3d attitude_rate;
	Vec3d attitude;
	Vec3d acc;
	Vec3d pos;
	Vec3d vel;
	double baro_altitude;
	double altitude;

	// PID targets
	float attCmd[3];
	double rateCmd[3];
	float posCmd[3];
	double velCmd[3];

	// PIDs
	bool enablePID;
	bool enableAttPID;
	bool enablePosPID;
	PID attPID[3];
	PID ratePID[3];
	PID posPID[3];
	PID velPID[3];

	// Interrupt System
	sc_event timerInterrupt;
	bool I2C_mag_update;
	bool I2C_acc_gyro_update;
	bool I2C_baro_update;

	PhysicsSim *sim;
	PhysicsObject *uav;

	SC_HAS_PROCESS(FlightController);

	FlightController(sc_module_name name, PhysicsSim* physim, PhysicsObject* vehicle) : sc_module(name) {
		SC_THREAD(mainI2C);
		SC_THREAD(mainIMU);
		SC_THREAD(mainTimer);
		SC_THREAD(mainPilot);

		this->sim = physim;
		this->uav = vehicle;

		// Initialize PIDs
		posPID[XAXIS].initialize(20, 20, 0, 20, 0, 0);
		posPID[YAXIS].initialize(20, 20, 0, 20, 0, 0);
		posPID[ZAXIS].initialize(20, 20, 0, 20, 0, 0);
		velPID[XAXIS].initialize(20, 20, 0, 20, 0, 0);
		velPID[YAXIS].initialize(20, 20, 0, 20, 0, 0);
		velPID[ZAXIS].initialize(20, 20, 0, 20, 0, 0);
		attPID[ROLL].initialize(3, 2, 0, M_PI_2, 0, 0);
		attPID[PITCH].initialize(3, 2, 0, M_PI_2, 0, 0);
		attPID[YAW].initialize(3, 2, 0, M_PI_2, 0, 0);
		ratePID[ROLL].initialize(5, 1, 0, 1, 0, 0);
		ratePID[PITCH].initialize(5, 1, 0, 1, 0, 0);
		ratePID[YAW].initialize(5, 1, 0, 1, 0, 0);
		enablePID = false;
		enablelog = false;
	}

	void mainIMU();
	void mainPilot();
	void mainI2C();
	void mainTimer();

	void setThrottle(double m1, double m2, double m3, double m4) {
		motor[0] = m1;
		motor[1] = m2;
		motor[2] = m3;
		motor[3] = m4;
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

	~FlightController() {
		disableLog();
	}
};

/////////////////////////////////////////////////
// System C Threads
/////////////////////////////////////////////////

// Temporary pilot command generation thread
void FlightController::mainPilot() {
	// Initialization
	enablePID = false;
	enableAttPID = true;
	enablePosPID = true;
	posCmd[XAXIS] = 0;	posCmd[YAXIS] = 0;	posCmd[ZAXIS] = 0;
	attCmd[ROLL] = 0;	attCmd[PITCH] = 0;	attCmd[YAW] = 0;
	rateCmd[ROLL] = 0;	rateCmd[PITCH] = 0;	rateCmd[YAW] = 0;

	// Flight sequence
	cout << "[" << sc_time_stamp() << "] Enabling PID control" << endl;
	enablePID = true;
	cout << "[" << sc_time_stamp() << "] Targeting altitude of 15m, yaw rate of 5deg/s" << endl;
	posCmd[ZAXIS] = 15;
	attCmd[YAW] = 45*M_PI/180;
	wait(5000, SC_MS);
	cout << "[" << sc_time_stamp() << "] Dropping altitude to 4m" << endl;
	posCmd[ZAXIS] = 4;
}

// Internal timer, generates interrupts at specific intervals
// Also updates the motor PWM outputs
void FlightController::mainTimer() {
	unsigned updateCycle;
	I2C_mag_update = false;
	I2C_acc_gyro_update = false;
	I2C_baro_update = false;
	while(true) {
		// 1000Hz timer
		writeAllMotors();
		wait(1, SC_MS);
		updateCycle++;
		if(updateCycle >= 1000)
			updateCycle = 0;
		if(I2C_mag_update || I2C_acc_gyro_update || I2C_baro_update)
			cout << "[" << sc_time_stamp() << "] I2C Bus error: unserviced request" << endl;
		if(I2C_mag_update || I2C_acc_gyro_update || I2C_baro_update)
			timerInterrupt.notify();
	}
}

// Implements I2C communication with DMA, places results into raw sensor value registers
// Does hardware averaging in DMA location, summing results and keeping track of
// result count.
// Uses ISF model with interrupt flags.
void FlightController::mainI2C() {
	unsigned i;
	unsigned buffer[16];
	while(true) {
		wait(timerInterrupt);

		// Read Accel/Gyro
		if(I2C_acc_gyro_update) {
			i2c_mst->i2c_read(LSM6DS3::I2C_ADDRESS, LSM6DS3::REG_GYRO_X_L, 12, buffer);
			cout << "Read from accel: " << sensor_raw.accel[0] << ", " << sensor_raw.accel[1] << ", " << sensor_raw.accel[2] << endl;
			for (i = 0; i < 3; i++) {
				sensor_raw.gyro[i] = (buffer[2*i + 1] << 8) + buffer[2*i];
				sensor_raw.accel[i] = (buffer[6 + (2*i + 1)] << 8) + buffer[6 + 2*i];
				sensor.accel[i] = sensor_raw.accel[i]/(LSM6DS3::ACC_SCALE_FACTOR) + ACCEL_SCALE_BIAS;
				sensor.gyro[i] = sensor_raw.gyro[i]/(LSM6DS3::GYRO_SCALE_FACTOR) + GYRO_SCALE_BIAS;
			}
			I2C_acc_gyro_update = false;
		}

		// Read mag
		if(I2C_mag_update) {
			i2c_mst->i2c_read(LIS3MDL::I2C_ADDRESS, LIS3MDL::REG_MAG_X_L, 6, buffer);
			cout << "Read from mag: " << sensor_raw.mag[0] << ", " << sensor_raw.mag[1] << ", " << sensor_raw.mag[2] << endl;
			for (i = 0; i < 3; i++) {
				sensor_raw.mag[i] = (buffer[2*i + 1] << 8) + buffer[2*i];
				sensor.mag[i] = sensor_raw.mag[i]/(LIS3MDL::MAG_SCALE_FACTOR) + MAG_SCALE_BIAS;
			}
			I2C_mag_update = false;
		}

		// Read temperature and pressure from barometer (100Hz), use first order filtering
		if(I2C_baro_update) {
			i2c_mst->i2c_read(LPS22HB::I2C_ADDRESS, LPS22HB::REG_PRES_XL, 5, buffer);
			cout << "Read from baro: " << sensor_raw.pressure << ", " << sensor_raw.temp << endl;
			cout << "Read from baro (buffer): " << buffer[0] << ", " << buffer[1] << ", " << buffer[2] << ", " << buffer[3] << ", " << buffer[4] << endl;
			sensor_raw.pressure = (buffer[2] << 16) + (buffer[1] << 8) + buffer[0];
			sensor_raw.temp = (buffer[4] << 8) + buffer[3];
			sensor.pressure = sensor_raw.pressure/(LPS22HB::PRESS_SCALE_FACTOR) + PRESSURE_SCALE_BIAS;
			sensor.temp = sensor_raw.temp/(LPS22HB::TEMP_SCALE_FACTOR) + TEMP_SCALE_BIAS;
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
	double dt = 0.001; // Delta time cycle for this update
	while(true) {
		// 1000 Hz
		wait(1, SC_MS);

		// TODO Do AHRS calculations to find the following:
		attitude = uav->attitude;
		attitude_rate = uav->attitude_rate;
		acc = uav->acceleration;
		vel = uav->velocity;
		pos = uav->position;

		// Apply PID Controllers
		if(enablePID) {
			// Compute error terms for roll and pitch in attitude mode, apply to PIDs as feedback
			if(enableAttPID) {
				error = wrapRadians(attCmd[ROLL] - attitude[ROLL]);
				rateCmd[ROLL] = attPID[ROLL].update(error, dt);
				error = wrapRadians(attCmd[PITCH] + attitude[PITCH]);
				rateCmd[PITCH] = attPID[PITCH].update(error, dt);
				error = wrapRadians(attCmd[YAW] - attitude[YAW]);
				rateCmd[YAW] = attPID[YAW].update(error, dt);
			}

			// Compute error terms for roll, pitch and yaw from rate, apply to PIDs as feedback
			error = rateCmd[ROLL] - attitude_rate[ROLL];
			ratePID[ROLL].update(error, dt);
			error = rateCmd[PITCH] + attitude_rate[PITCH];
			ratePID[PITCH].update(error, dt);
			error = rateCmd[YAW] - attitude_rate[YAW];
			ratePID[YAW].update(error, dt);

			// Compute error term for position based off estimation/integration, apply to PIDs as feedback
			if(enablePosPID) {
				error = posCmd[ZAXIS] - pos[ZAXIS];
				velCmd[ZAXIS] = posPID[ZAXIS].update(error, dt);
			}

			// Compute error term for velocity based off estimation/integration, apply to PIDs as feedback
			error = velCmd[ZAXIS] - vel[ZAXIS];
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

		#ifdef PID_ROLL
			motor[0] += ratePID[ROLL].state;
			motor[1] += -ratePID[ROLL].state;
			motor[2] += -ratePID[ROLL].state;
			motor[3] += ratePID[ROLL].state;
		#endif
		#ifdef PID_PITCH
			motor[0] += -ratePID[PITCH].state;
			motor[1] += -ratePID[PITCH].state;
			motor[2] += ratePID[PITCH].state;
			motor[3] += ratePID[PITCH].state;
		#endif
		#ifdef PID_YAW
			motor[0] += -ratePID[YAW].state;
			motor[1] += ratePID[YAW].state;
			motor[2] += -ratePID[YAW].state;
			motor[3] += ratePID[YAW].state;
		#endif

			// Motor log file output
			if(enablelog) {
				outfile << sc_time_stamp() << motor[0] << "\t" << motor [1] << "\t" << motor [2] << "\t" << motor [3] << endl;
			}
		}
	}
}


#endif
