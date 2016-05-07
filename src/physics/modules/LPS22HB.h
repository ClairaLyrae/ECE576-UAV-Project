#ifndef LPS22HB_H_
#define LPS22HB_H_

#include <systemc.h>
#include "../Physics.h"
#include "../../I2C.h"
#include "../../util/util.h"
#include <stdint.h>

class LPS22HB : public PhysicsComponent, public sc_module
{
private:
	double temp_err, press_err;
	uint32_t pressure;
	uint16_t temperature;
	uint8_t registers[128];
public:
	static const unsigned I2C_ADDRESS = 46;
	static const unsigned REG_PRES_XL = 0x28;
	static const unsigned REG_PRES_L = 0x29;
	static const unsigned REG_PRES_H = 0x2A;
	static const unsigned REG_TEMP_L = 0x2B;
	static const unsigned REG_TEMP_H = 0x2C;

	// Units in Pa
	static constexpr float PRESS_FULL_SCALE = 2160;
	static constexpr float PRESS_SCALE_FACTOR = 40.96;
	static constexpr float PRESS_NOISE_RMS = 0.75;

	// Units in C
	static constexpr float TEMP_FULL_SCALE = 2160;
	static constexpr float TEMP_SCALE_FACTOR = 40.96;
	static constexpr float TEMP_NOISE_RMS = 0.75;

	sc_port<i2c_slv_if> i2c_slv;

	NoiseGenerator tnoise;
	NoiseGenerator pnoise;

	SC_HAS_PROCESS(LPS22HB);

	LPS22HB(sc_module_name name) : sc_module(name), tnoise(0, TEMP_NOISE_RMS), pnoise(0, PRESS_NOISE_RMS) {
		SC_THREAD(main);
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
		pressure = (sim.getPressure(parent.position.mData[2]) + pnoise.generate())*PRESS_SCALE_FACTOR;
		temperature = (sim.getTemperature(parent.position.mData[2]) - 273.15 + tnoise.generate())*PRESS_SCALE_FACTOR;

		// Set sensor registers to calculated value
		registers[REG_PRES_XL] = pressure & 0xFF;
		registers[REG_PRES_L] = (pressure >> 8) & 0xFF;
		registers[REG_PRES_H] = (pressure >> 16) & 0xFF;
		registers[REG_TEMP_L] = pressure & 0xFF;
		registers[REG_TEMP_H] = (pressure >> 8) & 0xFF;
	}
};

#endif
