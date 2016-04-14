#ifndef PWM_H_
#define PWM_H_

#include <systemc.h>

// PWM Output Interface
class PWM_out : virtual public sc_interface
{
public:
   virtual void writePWM(double duty);
};

// PWM Receiver Interface
class PWM_in : virtual public sc_interface
{
public:
   virtual double readPWM();
   virtual double listenPWM();
};

// PWM Bus Interface
class PWM_bus : public sc_module, public PWM_in, public PWM_out
{
public:
	double period_ns;
	double duty_new;
	double duty;

	sc_event duty_write_event;
	sc_event duty_change_event;

	SC_HAS_PROCESS(PWM_bus);

	PWM_bus(sc_module_name name, double period) : sc_module(name) {
		SC_THREAD(main);
		this->duty = 0;
		this->duty_new = 0;
		this->period_ns = period;
	}

	void main() {
		while(true) {
			wait(duty_write_event);
			wait(period_ns, SC_NS);
			duty = duty_new;
			duty_change_event.notify();
		}
	}

	void writePWM(double dc) {
		duty_new = dc;
		duty_write_event.notify();
	}

	double readPWM() {
		return duty;
	}

	double listenPWM() {
		wait(duty_change_event);
		return duty;
	}
};

#endif
