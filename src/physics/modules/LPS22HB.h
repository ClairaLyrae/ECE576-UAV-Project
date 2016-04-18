#ifndef LPS22HB_H_
#define LPS22HB_H_

#include <systemc.h>
#include "../Physics.h"
#include "../../I2C.h"

// Units in Pa
#define PRESS_FULL_SCALE 2160
#define PRESS_SCALE_FACTOR 40.96
#define PRESS_NOISE_RMS 0.75

#define PRESS_OUT_H 0x2A
#define PRESS_OUT_L 0x29
#define PRESS_OUT_XL 0x28

class LPS22HB : public PhysicsComponent, public sc_module
{
private:
	long pressure;
	unsigned registers[128];
public:
	const unsigned I2C_ADDRESS = 46;
	sc_port<iic_slv_if> i2c_slv;

	SC_HAS_PROCESS(LPS22HB);

	LPS22HB(sc_module_name name) : sc_module(name) {
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
		pressure = sim.pressureAtAltitude(parent.position.mData[2])*PRESS_SCALE_FACTOR;

		// Set sensor registers to calculated value
		registers[PRESS_OUT_XL] = pressure & 0xFF;
		registers[PRESS_OUT_L] = (pressure >> 8) & 0xFF;
		registers[PRESS_OUT_H] = (pressure >> 16) & 0xFF;
	}
};

#endif
