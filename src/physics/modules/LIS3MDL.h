#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include <systemc.h>
#include "../Physics.h"
#include "../../I2C.h"
#include <stdint.h>

class LIS3MDL : public PhysicsComponent, public sc_module
{
private:
	uint16_t mag[3];
	uint8_t registers[128];
public:
	static const unsigned I2C_ADDRESS = 54;
	static const unsigned REG_MAG_X_L = 0x28;
	static const unsigned REG_MAG_X_H = 0x29;
	static const unsigned REG_MAG_Y_L = 0x2A;
	static const unsigned REG_MAG_Y_H = 0x2B;
	static const unsigned REG_MAG_Z_L = 0x2C;
	static const unsigned REG_MAG_Z_H = 0x2D;

	// Units in Gauss
	static constexpr float MAG_FULL_SCALE = 4;			// Gs
	static constexpr float MAG_SCALE_FACTOR = 6842;	// LSB/Gs
	static constexpr float MAG_NOISE_RMS = 0.75;

	sc_port<i2c_slv_if> i2c_slv;

	SC_HAS_PROCESS(LIS3MDL);

	LIS3MDL(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
		//SC_THREAD(tick);
	}

	void tick() {
		while(true) {
			wait(1000, SC_MS);
			cout << sc_time_stamp() << ":\tLIS3MDL Mag = " << mag[0] << ", " << mag[1] << ", " << mag[2] << endl;
		}
	}

	void main() {
		// Respond to I2C bus
		uint8_t req_addr, reg_loc, i;
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

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Update sensor data from physics object
		mag[0] = sim.getMagneticField().mData[0]*MAG_SCALE_FACTOR;
		mag[1] = sim.getMagneticField().mData[1]*MAG_SCALE_FACTOR;
		mag[2] = sim.getMagneticField().mData[2]*MAG_SCALE_FACTOR;

		// Set sensor registers to calculated value
		registers[REG_MAG_X_L] = mag[0] & 0xFF;
		registers[REG_MAG_X_H] = (mag[0] >> 8) & 0xFF;
		registers[REG_MAG_Y_L] = mag[1] & 0xFF;
		registers[REG_MAG_Y_H] = (mag[1] >> 8) & 0xFF;
		registers[REG_MAG_Z_L] = mag[2] & 0xFF;
		registers[REG_MAG_Z_H] = (mag[2] >> 8) & 0xFF;
	}
};

#endif
