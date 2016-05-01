#ifndef LSM6DS3_H_
#define LSM6DS3_H_

#include <systemc.h>
#include "../Physics.h"
#include "../../I2C.h"

class LSM6DS3 : public PhysicsComponent, public sc_module
{
private:
	long acc_x, acc_y, acc_z;
	long gyro_x, gyro_y, gyro_z;
	unsigned registers[128];
public:
	static const unsigned I2C_ADDRESS = 53;
	static const unsigned REG_GYRO_X_L = 0x22;
	static const unsigned REG_GYRO_X_H = 0x23;
	static const unsigned REG_GYRO_Y_L = 0x24;
	static const unsigned REG_GYRO_Y_H = 0x25;
	static const unsigned REG_GYRO_Z_L = 0x26;
	static const unsigned REG_GYRO_Z_H = 0x27;
	static const unsigned REG_ACC_X_L = 0x28;
	static const unsigned REG_ACC_X_H = 0x29;
	static const unsigned REG_ACC_Y_L = 0x2A;
	static const unsigned REG_ACC_Y_H = 0x2B;
	static const unsigned REG_ACC_Z_L = 0x2C;
	static const unsigned REG_ACC_Z_H = 0x2D;

	// Units in m/s^2
	static constexpr float ACC_FULL_SCALE = 39.2266;			// m/s^2
	static constexpr float ACC_SCALE_FACTOR = 835.83296145; 	// LSB/(m/s^2)
	static constexpr float ACC_NOISE_DENSITY = 0.001765197; 	// 1 / sqrt(Hz)
	static constexpr float ACC_NOISE_RMS = 0.04314926;

	// Units in deg/s
	static constexpr float GYRO_FULL_SCALE = 245;			// deg/s
	static constexpr float GYRO_SCALE_FACTOR = 114.28571428;	// LSB/(deg/s)
	static constexpr float GYRO_NOISE_DENSITY = 0.007; 		// 1 / sqrt(Hz)
	static constexpr float GYRO_NOISE_RMS = 0.14;

	sc_port<i2c_slv_if> i2c_slv;

	SC_HAS_PROCESS(LSM6DS3);

	LSM6DS3(sc_module_name name) : sc_module(name) {
		//SC_THREAD(main);
		//SC_THREAD(tick);
	}

	void tick() {
		while(true) {
			wait(200, SC_MS);
			cout << sc_time_stamp() << ":\tLSM6DS3 Gyro = " << gyro_x << ", " << gyro_y << ", " << gyro_z << endl;
			cout << sc_time_stamp() << ":\tLSM6DS3 Accel = " << acc_x << ", " << acc_y << ", " << acc_z << endl;
		}
	}

	void main() {
		// Respond to I2C bus
		unsigned req_addr, reg_loc, i;
		bool stop, req_rw;
		while(true) {
			i = 0;
			req_addr = 0;
			stop = false;
			i2c_slv->i2c_listen(req_addr, req_rw);
			if(req_addr == I2C_ADDRESS) {
				i2c_slv->slv_i2c_ack();
				if(!req_rw) {
					i2c_slv->slv_i2c_rec(reg_loc,stop);
					while(!stop) {
						i2c_slv->slv_i2c_rec(registers[reg_loc + i],stop);
						i++;
					}
				}
				else {
					while(!stop) {
						i2c_slv->slv_i2c_send(registers[reg_loc + i],stop);
						i++;
					}
				}
			}
		}
	}


	Vec3f getAcceleration() {
		return Vec3f();
	}

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		acc_x = (long)(parent.acceleration.mData[0]*ACC_SCALE_FACTOR);
		acc_y = (long)(parent.acceleration.mData[1]*ACC_SCALE_FACTOR);
		acc_z = (long)(parent.acceleration.mData[2]*ACC_SCALE_FACTOR);
		gyro_x = (long)(parent.rotation.mData[0]*GYRO_SCALE_FACTOR);
		gyro_y = (long)(parent.rotation.mData[2]*GYRO_SCALE_FACTOR);
		gyro_z = (long)(parent.rotation.mData[3]*GYRO_SCALE_FACTOR);

		// Calculated values for register
		registers[REG_ACC_X_L] = (acc_x) & 0xFF;
		registers[REG_ACC_X_H] = (acc_x >> 8) & 0xFF;
		registers[REG_ACC_Y_L] = (acc_y) & 0xFF;
		registers[REG_ACC_Y_H] = (acc_y >> 8) & 0xFF;
		registers[REG_ACC_Z_L] = (acc_z) & 0xFF;
		registers[REG_ACC_Z_H] = (acc_z >> 8) & 0xFF;
		registers[REG_GYRO_X_L] = (gyro_x) & 0xFF;
		registers[REG_GYRO_X_H] = (gyro_x >> 8) & 0xFF;
		registers[REG_GYRO_Y_L] = (gyro_y) & 0xFF;
		registers[REG_GYRO_Y_H] = (gyro_y >> 8) & 0xFF;
		registers[REG_GYRO_Z_L] = (gyro_z) & 0xFF;
		registers[REG_GYRO_Z_H] = (gyro_z >> 8) & 0xFF;
	}
};

#endif
