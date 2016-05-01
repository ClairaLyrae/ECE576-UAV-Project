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

#define M_PI 3.14159265359
#define M_PI_2 1.57079632679

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#define ROLL 0
#define PITCH 1
#define YAW 2

#define MIN_THROTTLE 0
#define MAX_THROTTLE 100

#define ACCEL_SCALE_BIAS 0
#define GYRO_SCALE_BIAS 0
#define MAG_SCALE_BIAS 0
#define TEMP_SCALE_BIAS 0
#define PRESSURE_SCALE_BIAS 0

#define PITCH_LIMIT 20
#define ROLL_LIMIT 20
#define PITCH_COMPENSATION_GAIN 0
#define ROLL_COMPENSATION_GAIN 0


#define AIR_MOLAR_MASS 0.0289644	// kg/mol
#define TEMP_LAPSE_RATE 0.0065		// K/m
#define GAS_CONSTANT 8.31447		// J/(mol*K)
#define EARTH_GRAVITY 9.80665		// m/s^2
#define AIR_DENSITY 1.225			//
#define STP_TEMP 288.15				// K
#define STP_PRESSURE 101325			// Pa

using namespace gmtl;

// Hardware Module
class FlightController : public sc_module
{
public:
	sc_port<i2c_mst_if> i2c_mst;
	sc_port<PWM_out> pwm_out_1;
	sc_port<PWM_out> pwm_out_2;
	sc_port<PWM_out> pwm_out_3;
	sc_port<PWM_out> pwm_out_4;

	// Motor output values
	int motor[4];

	// Sensor data
	Sensors sensor_raw;
	Sensors sensor_sum;
	int count_accel;
	int count_gyro;
	int count_mag;
	int count_baro;
	Sensors sensor;
	double accel_e[3];
	double baro_altitude;

	// Final command values
	float attCmd[3];
	float throttleCmd;

	// PID targets
	float headingReference;
	float altitudeHoldReference;
	float throttleReference;

	// PIDs
	PID attPID[3];
	PID ratePID[3];
	PID hPID;
	PID hDotPID;

	// Submodules
	AHRS ahrs;
	AltitudeEstimator altitude_est;

	PhysicsSim *sim;
	PhysicsObject *uav;

	bool enablePID = false;

	SC_HAS_PROCESS(FlightController);

	FlightController(sc_module_name name, PhysicsSim* physim, PhysicsObject* vehicle) : sc_module(name) {
		//SC_THREAD(mainCAN);
		SC_THREAD(mainPWM);
		//SC_THREAD(mainI2C);
		SC_THREAD(mainIMU);

		this->sim = physim;
		this->uav = vehicle;
		// Initialize PIDs
		hPID.initialize(2, 0, 0, 10, 0, 0);
		hDotPID.initialize(450, 0, 0, 60, 0, 0);
		attPID[ROLL].initialize(1, 0, 0, 1, 0, 0);
		attPID[PITCH].initialize(2, 0, 0, 2, 0, 0);
		attPID[YAW].initialize(3, 0, 0, M_PI_2, 0, 0);
		ratePID[ROLL].initialize(175, 100, 2, 300, 0, 0);
		ratePID[PITCH].initialize(175, 100, 2, 300, 0, 0);
		ratePID[YAW].initialize(175, 100, 2, 300, 0, 0);
	}

	void mainCAN() {
		while(true) {
			// Listen for broadcasts. Ack and reply with attitude if necessary, otherwise absorb info.

			// receive flight commands (~50Hz)
			// receive gps broadcast (~10Hz)
			// receive attitude requests and respond
		}
	}

	void mainPWM() {
//		cout << "Set throttle to 100%" << endl;
//		setThrottle(1.0, 1.0, 1.0, 1.0);
//		wait(1000, SC_MS);
//		cout << "Set throttle to slight turn" << endl;
//		setThrottle(0.5, 0.5, 0.482, 0.482);
//		wait(200, SC_MS);
//		cout << "Set throttle to slight opposite turn, zero rotation" << endl;
//		setThrottle(0.482, 0.482, 0.5, 0.5);
//		wait(200, SC_MS);
//		cout << "Set throttle to hover" << endl;
//		setThrottle(0.482, 0.482, 0.482, 0.482);
//		wait(15000, SC_MS);
//		setThrottle(0.0, 0.0, 0.0, 0.0);


		cout << "Set throttle to  100%" << endl;
		setThrottle(1.0, 1.0, 1.0, 1.0);
		wait(5000, SC_MS);
		cout << "Set throttle to arbitrary values (highly asymmetric)" << endl;
		setThrottle(0.85, 0.2, 0.53, 0.1);
		cout << "Throttle set" << endl;
		wait(100, SC_MS);
		enablePID = true;

		while(true) {
			wait(1, SC_MS);
			setThrottle(motor[0], motor[1], motor[2], motor[3]);
		}
	}

	// Implements I2C communication with DMA, places results into raw sensor value registers
	// Does hardware averaging in DMA location, summing results and keeping track of
	// result count
	void mainI2C() {
		unsigned i;
		unsigned buffer[16];
		while(true) {
			wait(500, SC_MS);

			// Read mag
			i2c_mst->i2c_read(LIS3MDL::I2C_ADDRESS, LIS3MDL::REG_MAG_X_L, 6, buffer);
			cout << "Read from mag: " << sensor_raw.mag[0] << ", " << sensor_raw.mag[1] << ", " << sensor_raw.mag[2] << endl;
			for (i = 0; i < 3; i++) {
				sensor_raw.mag[i] = (buffer[2*i + 1] << 8) + buffer[2*i];
				sensor.mag[i] = sensor_raw.mag[i]/(LIS3MDL::MAG_SCALE_FACTOR) + MAG_SCALE_BIAS;
			}

			// Read temperature and pressure from barometer (100Hz), use first order filtering
			i2c_mst->i2c_read(LPS22HB::I2C_ADDRESS, LPS22HB::REG_PRES_XL, 5, buffer);
			cout << "Read from baro: " << sensor_raw.pressure << ", " << sensor_raw.temp << endl;
			cout << "Read from baro (buffer): " << buffer[0] << ", " << buffer[1] << ", " << buffer[2] << ", " << buffer[3] << ", " << buffer[4] << endl;
			sensor_raw.pressure = (buffer[2] << 16) + (buffer[1] << 8) + buffer[0];
			sensor_raw.temp = (buffer[4] << 8) + buffer[3];
			sensor.pressure = sensor_raw.pressure/(LPS22HB::PRESS_SCALE_FACTOR) + PRESSURE_SCALE_BIAS;
			sensor.temp = sensor_raw.temp/(LPS22HB::TEMP_SCALE_FACTOR) + TEMP_SCALE_BIAS;
			baro_altitude = (STP_TEMP/TEMP_LAPSE_RATE) * (1.0f - pow(sensor.pressure / STP_PRESSURE, (GAS_CONSTANT*TEMP_LAPSE_RATE)/(EARTH_GRAVITY*AIR_MOLAR_MASS)));


			// Read Accel/Gyro
			i2c_mst->i2c_read(LSM6DS3::I2C_ADDRESS, LSM6DS3::REG_GYRO_X_L, 12, buffer);
			cout << "Read from accel: " << sensor_raw.accel[0] << ", " << sensor_raw.accel[1] << ", " << sensor_raw.accel[2] << endl;
			for (i = 0; i < 3; i++) {
				sensor_raw.gyro[i] = (buffer[2*i + 1] << 8) + buffer[2*i];
				sensor_raw.accel[i] = (buffer[6 + (2*i + 1)] << 8) + buffer[6 + 2*i];
				sensor.accel[i] = sensor_raw.accel[i]/(LSM6DS3::ACC_SCALE_FACTOR) + ACCEL_SCALE_BIAS;
				sensor.gyro[i] = sensor_raw.gyro[i]/(LSM6DS3::GYRO_SCALE_FACTOR) + GYRO_SCALE_BIAS;
			}
		}
	}

	void mainIMU() {
		unsigned i;

		while(true) {
			wait(2, SC_MS);

			Vec3d uav_acc;
			Vec3d uav_gyro;
			Vec3d uav_mag;
			xform(uav_acc, uav->orientation, uav->acceleration);
			xform(uav_gyro, uav->orientation, uav->rotation);
			xform(uav_mag, uav->orientation, sim->getMagneticField());
			sensor.accel[0] = uav_acc.mData[0];
			sensor.accel[1] = uav_acc.mData[1];
			sensor.accel[2] = uav_acc.mData[2];
			sensor.gyro[0] = uav_gyro.mData[0];
			sensor.gyro[1] = uav_gyro.mData[1];
			sensor.gyro[2] = uav_gyro.mData[2];
			sensor.pressure = sim->getPressure(uav->position.mData[2]);
			sensor.temp = sim->getTemperature(uav->position.mData[2]);
			baro_altitude = (STP_TEMP/TEMP_LAPSE_RATE)*(1 - pow((sensor.pressure/STP_PRESSURE), (GAS_CONSTANT*TEMP_LAPSE_RATE)/(EARTH_GRAVITY*AIR_MOLAR_MASS)));

			cout << "-----------------------------------------------" << endl;
			cout << "Sensor readings at time t=" << sc_time_stamp() << endl;
			cout << "-----------------------------------------------" << endl;
			cout << "Temperature = " << sensor.temp << endl;
			cout << "Pressure = " << sensor.pressure << endl;
			cout << "Baro Altitude = " << baro_altitude << endl;
			cout << "Gyro = " << sensor.gyro[0] << ", " << sensor.gyro[1] << ", " << sensor.gyro[2] << endl;
			cout << "Accel = " << sensor.accel[0] << ", " << sensor.accel[1] << ", " << sensor.accel[2] << endl;
			cout << "Mag = " << sensor.mag[0] << ", " << sensor.mag[1] << ", " << sensor.mag[2] << endl;

			cout << "Phys Accel = " << uav->acceleration[0] << ", " << uav->acceleration[1] << ", " << uav->acceleration[2] << endl;
			if(enablePID) {
				double dt500Hz = 0.002; // Delta time cycle for this update

				// Calculate acceleration data relative to earth rather than sensor axis
				ahrs.computeEarthAccel(sensor);
				accel_e[XAXIS] = ahrs.e_accel.mData[XAXIS];
				accel_e[YAXIS] = ahrs.e_accel.mData[YAXIS];
				accel_e[ZAXIS] = ahrs.e_accel.mData[ZAXIS];
				cout << "AHRS Accel = " << ahrs.e_accel[0] << ", " << ahrs.e_accel[1] << ", " << ahrs.e_accel[2] << endl;
				// Update altitude estimation filter (compares altimeter with integrated vertical acceleration)
				altitude_est.update(baro_altitude, accel_e[ZAXIS], dt500Hz);
				// Calculate attitude
				ahrs.update(sensor, dt500Hz, true);

				// Calculate and update motor/servo outputs
				applyPID(dt500Hz);
				setThrottle(motor[0], motor[1], motor[2], motor[3]);
			}

		}
	}

	void applyPID(float dt)
	{
		unsigned i;
		float error;
		float tempAttCompensation;
		float verticalVelocityCmd;
		float rateCmd[3];

		// Compute error terms for roll and pitch in attitude mode (User input compared to vehicle attitude)
		// Apply error terms to PID controllers as feedback
		error = wrapRadians(attCmd[ROLL] - ahrs.attitude[ROLL]);
		attPID[ROLL].update(error, dt);
		error = wrapRadians(attCmd[PITCH] + ahrs.attitude[PITCH]);
		attPID[PITCH].update(error, dt);

		// Take control input from output of attitude PIDs
		rateCmd[ROLL ] = attPID[ROLL ].state;
		rateCmd[PITCH] = attPID[PITCH].state;

		// Heading Hold. Compute error term for yaw, apply to PID, and use PID calculation for output
		error = wrapRadians(headingReference - ahrs.attitude[YAW]);
		rateCmd[YAW] = attPID[YAW].update(error, dt);

		// Compute error terms for roll, pitch and yaw from rate OR attitude inputs found above
		// Apply error terms to rate PID controllers as feedback based on gyro data
		// Rate PIDs based off error on gyro only
		error = rateCmd[ROLL] - sensor.gyro[ROLL];
		ratePID[ROLL].update(error, dt);
		error = rateCmd[PITCH] + sensor.gyro[PITCH];
		ratePID[PITCH].update(error, dt);
		error = rateCmd[YAW] - sensor.gyro[YAW];
		ratePID[YAW].update(error, dt);

		// Compute error term for altitude position based off height estimation
		// Use as feedback to vertical position hold PID, which serves as input to vertical velocity PID
		// altitudeHoldReference depends on particular type of altitude hold enabled (fixed, at engagement, or throttle inactive)
		error = altitudeHoldReference - altitude_est.h;
		verticalVelocityCmd = hPID.update(error, dt);

		// Compute error term for vertical velocity hold PID based of estimated vertical velocity (hDot)
		// Use as feedback to vertical velocity hold PID
		error = verticalVelocityCmd - altitude_est.hDot;
		throttleCmd = throttleReference + hDotPID.update(error, dt);

		// Get Roll Angle, Constrain to +/-20 degrees (default)
		tempAttCompensation = constrain(ahrs.attitude[ROLL], -ROLL_LIMIT,  ROLL_LIMIT);
		// Compute Cosine of Roll Angle and Multiply by Att-Alt Gain
		tempAttCompensation = ROLL_COMPENSATION_GAIN / cosf(tempAttCompensation);

		// Apply Roll Att Compensation to Throttle Command
		throttleCmd *= tempAttCompensation;

		// Get Pitch Angle, Constrain to +/-20 degrees (default)
		tempAttCompensation = constrain(ahrs.attitude[PITCH], -PITCH_LIMIT,  PITCH_LIMIT);
		// Compute Cosine of Pitch Angle and Multiply by Att-Alt Gain
		tempAttCompensation = PITCH_COMPENSATION_GAIN / cosf(tempAttCompensation);

		// Apply Pitch Att Compensation to Throttle Command
		throttleCmd *= tempAttCompensation;

		// Mixing table (assign motor outptus to achieve desired attitude change)
		motor[0] = (ratePID[ROLL].state*(1) + ratePID[PITCH].state*(-1) + ratePID[YAW].state*(-1) + throttleCmd*(1));     // Front left (CW)
		motor[1] = (ratePID[ROLL].state*(-1) + ratePID[PITCH].state*(-1) + ratePID[YAW].state*(1) + throttleCmd*(1));     // Front right (CCW)
		motor[2] = (ratePID[ROLL].state*(-1) + ratePID[PITCH].state*(1) + ratePID[YAW].state*(-1) + throttleCmd*(1));     // Back right (CW)
		motor[3] = (ratePID[ROLL].state*(1) + ratePID[PITCH].state*(1) + ratePID[YAW].state*(1) + throttleCmd*(1));     // Back left  (CCW)

		// Rescale if motor reaches max to keep gyro correction working
		int maxMotor = motor[0];
		for (i = 1; i < 4; i++)
			if (motor[i] > maxMotor)
				maxMotor = motor[i];

		// Constrain motor values
		for (i = 0; i < 4; i++)
		{
			if(maxMotor > MAX_THROTTLE)
				motor[i] -= maxMotor - MAX_THROTTLE;
			motor[i] = constrain(motor[i], MIN_THROTTLE, MAX_THROTTLE);
		}
	}

	void setThrottle(double m1, double m2, double m3, double m4) {
		pwm_out_1->writePWM(m1);
		pwm_out_2->writePWM(m2);
		pwm_out_3->writePWM(m3);
		pwm_out_4->writePWM(m4);
	}

	void setThrottle(double t) {
		pwm_out_1->writePWM(t);
		pwm_out_2->writePWM(t);
		pwm_out_3->writePWM(t);
		pwm_out_4->writePWM(t);
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

	float constrain(float input, float minValue, float maxValue)
	{
	    if (input < minValue)
	        return minValue;
	    else if (input > maxValue)
	        return maxValue;
	    else
	        return input;
	}
};

#endif
