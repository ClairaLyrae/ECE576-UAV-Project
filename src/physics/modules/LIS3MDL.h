#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include <systemc.h>
#include "../Physics.h"
#include "../../I2C.h"

#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

class LIS3MDL : public PhysicsComponent, public sc_module
{
private:
	unsigned registers[128];
public:
	const unsigned I2C_ADDRESS = 53;
	sc_port<iic_slv_if> i2c_slv;

	SC_HAS_PROCESS(LIS3MDL);

	LIS3MDL(sc_module_name name) : sc_module(name) {
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

	void update(double delta, PhysicsSim &sim, PhysicsObject &parent) {
		// Update sensor data from physics object
		// TODO Calculate magnetometer values
	}
};

#endif
