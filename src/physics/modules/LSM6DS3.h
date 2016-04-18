#ifndef LSM6DS3_H_
#define LSM6DS3_H_

#include <systemc.h>
#include "../Physics.h"
#include "../../I2C.h"

// Units in m/s^2
#define ACC_FULL_SCALE 156.9064
#define ACC_SCALE_FACTOR 0.0047856452
#define ACC_NOISE_DENSITY 0.001765197 // 1 / sqrt(Hz)
#define ACC_NOISE_RMS 0.04314926

// Units in deg/s
#define GYRO_FULL_SCALE 245
#define GYRO_SCALE_FACTOR 0.00875
#define GYRO_NOISE_RMS 0.14
#define GYRO_NOISE_DENSITY 0.007 // 1 / sqrt(Hz)

// Register addresses
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

class LSM6DS3 : public PhysicsComponent, public sc_module
{
private:
	long acc_x, acc_y, acc_z;
	long gyro_x, gyro_y, gyro_z;
	unsigned registers[128];
public:
	const unsigned I2C_ADDRESS = 53;
	sc_port<iic_slv_if> i2c_slv;

	SC_HAS_PROCESS(LSM6DS3);

	LSM6DS3(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
	}

	void main() {
		// Respond to I2C bus
		unsigned req_addr, reg_loc, i;
		bool stop, req_rw;
		while(true) {
			i = 0;
			req_addr = 0;
			stop = false;
			i2c_slv->iic_listen(req_addr, req_rw);
			if(req_addr == I2C_ADDRESS) {
				i2c_slv->slv_iic_ack();
				if(!req_rw) {
					i2c_slv->slv_iic_rec(reg_loc,stop);
					while(!stop) {
						i2c_slv->slv_iic_rec(registers[reg_loc + i],stop);
						i++;
					}
				}
				else {
					while(!stop) {
						i2c_slv->slv_iic_send(registers[reg_loc + i],stop);
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
		acc_x = parent.acceleration.mData[0]*ACC_SCALE_FACTOR;
		acc_y = parent.acceleration.mData[1]*ACC_SCALE_FACTOR;
		acc_z = parent.acceleration.mData[2]*ACC_SCALE_FACTOR;
		gyro_x = parent.rotation.mData[0]*GYRO_SCALE_FACTOR;
		gyro_y = parent.rotation.mData[2]*GYRO_SCALE_FACTOR;
		gyro_z = parent.rotation.mData[3]*GYRO_SCALE_FACTOR;

		// Calculated values for register
		registers[OUTX_L_XL] = (acc_x) & 0xFF;
		registers[OUTX_H_XL] = (acc_x >> 8) & 0xFF;
		registers[OUTY_L_XL] = (acc_y) & 0xFF;
		registers[OUTY_H_XL] = (acc_y >> 8) & 0xFF;
		registers[OUTZ_L_XL] = (acc_z) & 0xFF;
		registers[OUTZ_H_XL] = (acc_z >> 8) & 0xFF;
		registers[OUTX_L_G] = (gyro_x) & 0xFF;
		registers[OUTX_H_G] = (gyro_x >> 8) & 0xFF;
		registers[OUTY_L_G] = (gyro_y) & 0xFF;
		registers[OUTY_H_G] = (gyro_y >> 8) & 0xFF;
		registers[OUTZ_L_G] = (gyro_z) & 0xFF;
		registers[OUTZ_H_G] = (gyro_z >> 8) & 0xFF;
	}
};

#endif
