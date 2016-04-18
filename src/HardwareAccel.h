#ifndef HARDWARE_ACCEL_H_
#define HARDWARE_ACCEL_H_

#include <systemc.h>
#include "PWM.h"
#include "I2C.h"

#define TRAJ_CHAOS 0
#define TRAJ_HORIZ 1
#define TRAJ_BALLISTIC 2
#define TRAJ_UNFLIP 3
#define TRAJ_TYPE TRAJ_HORIZ

// Hardware Module
class HardwareAccel : public sc_module
{
public:
	sc_port<iic_mst_if> i2c_mst;
	sc_port<PWM_out> pwm_out_1;
	sc_port<PWM_out> pwm_out_2;
	sc_port<PWM_out> pwm_out_3;
	sc_port<PWM_out> pwm_out_4;

	SC_HAS_PROCESS(HardwareAccel);

	HardwareAccel(sc_module_name name) : sc_module(name) {
		SC_THREAD(main);
	}

	void main() {
		switch(TRAJ_TYPE) {
		case TRAJ_UNFLIP:
			setThrottle(1.0, 1.0, 1.0, 1.0);
			wait(3000, SC_MS);
			setThrottle(1.0, 1.0, 0.0, 0.0);
			wait(6000, SC_MS);
			setThrottle(0.0, 0.0, 1.0, 1.0);
			wait(6000, SC_MS);
			setThrottle(1.0, 1.0, 1.0, 1.0);
			break;
		case TRAJ_CHAOS:
			setThrottle(1.0, 1.0, 1.0, 1.0);
			wait(3000, SC_MS);
			setThrottle(0.85, 0.2, 0.53, 0.1);
			break;
		case TRAJ_HORIZ:
			setThrottle(1.0, 1.0, 1.0, 1.0);
			wait(1000, SC_MS);
			setThrottle(0.5, 0.5, 0.482, 0.482);
			wait(200, SC_MS);
			setThrottle(0.482, 0.482, 0.5, 0.5);
			wait(200, SC_MS);
			setThrottle(0.482, 0.482, 0.482, 0.482);
			wait(15000, SC_MS);
			setThrottle(0.0, 0.0, 0.0, 0.0);
			break;
		case TRAJ_BALLISTIC:
			setThrottle(1, 1, 0.9, 0.9);
			wait(1000, SC_MS);
			setThrottle(0.9, 0.9, 1.0, 1.0);
			wait(1000, SC_MS);
			setThrottle(0.0, 0.0, 0.0, 0.0);
			wait(1000, SC_MS);
			break;
		}


		return;
	}

	void setThrottle(double m1, double m2, double m3, double m4) {
		pwm_out_1->writePWM(m1);
		pwm_out_2->writePWM(m2);
		pwm_out_3->writePWM(m3);
		pwm_out_4->writePWM(m4);
	}
};

#endif
