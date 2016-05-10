#ifndef I2C_H_
#define I2C_H_

#include <systemc.h>
#include <stdint.h>

#define I2C_WRITE false
#define I2C_READ true

class i2c_mst_if : virtual public sc_interface
{
public:
	virtual void i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) = 0;
	virtual void i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) = 0;
	virtual void mst_i2c_send(uint8_t data, bool stop) = 0;
	virtual void mst_i2c_rec(uint8_t &data, bool stop) = 0;
};

class i2c_slv_if : virtual public sc_interface
{
public:
	virtual void i2c_listen(uint8_t &addr, bool &rw) = 0;
	virtual void slv_i2c_ack() = 0;
	virtual void slv_i2c_rec(uint8_t &data, bool &stop) = 0;
	virtual void slv_i2c_send(uint8_t data, bool &stop) = 0;
};

class i2c_bus : public sc_module, public i2c_mst_if, public i2c_slv_if
{
private:
	sc_event i2c_ack, data_ready, slv_add, slv_ready;
	uint8_t transfer, dest, end;
	bool rdnwr, display;
	sc_mutex i2c_active;
	double period_ns;

	// Stats
	unsigned long message_count;
	sc_time time_start, time_active;

	void i2c_start(uint8_t addr, bool rw)
	{
		i2c_active.lock();
		dest = addr;
		rdnwr = rw;
		wait(9*period_ns, SC_NS);
		slv_add.notify();
		wait(i2c_ack);
		return;
	}

	void i2c_continue(uint8_t addr, bool rw)
	{
		dest = addr;
		rdnwr = rw;
		wait(9*period_ns,SC_NS);
		slv_add.notify();
		wait(i2c_ack);
		return;
	}
public:
	SC_HAS_PROCESS(i2c_bus);

	i2c_bus(sc_module_name name, double freq) : sc_module(name)
	{
		SC_THREAD(main);
		setFrequency(freq);
		transfer = 0;
		dest = 0;
		end = 0;
		rdnwr = false;
		display = false;
		time_start = SC_ZERO_TIME;
		time_active = SC_ZERO_TIME;
	}

	void main() {
		time_start = sc_time_stamp();
		time_active = SC_ZERO_TIME;
	}

	double utilization() {
		sc_time tot = sc_time_stamp() - time_start;
		return time_active.to_seconds()/tot.to_seconds();
	}

	long numTransactions() {
		return message_count;
	}

	void setFrequency(double freq) {
		period_ns = (1000000000.0/freq);
	}

	void showTraffic(bool b) {
		display = b;
	}

	void i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
		sc_time time_trans;
		time_trans = sc_time_stamp();
		uint8_t i = 0;
		if(display)
			cout << "[" << sc_time_stamp() << "] I2C write to (add=" << addr << ", reg=" << reg << ")" << endl;
		i2c_start(addr, I2C_WRITE);
		mst_i2c_send(reg, true);
		for(i = 0; i < len; i++) {
			mst_i2c_send(data[i], (i == len-1) ? true : false);
		}
		wait(period_ns, SC_NS);
		i2c_active.unlock();
		time_active += sc_time_stamp() - time_trans;
		message_count++;
	}

	void i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
		sc_time time_trans;
		time_trans = sc_time_stamp();
		uint8_t i = 0;
		if(display)
			cout << "[" << sc_time_stamp() << "] I2C read from (add=" << (unsigned)addr << ", reg=" << (unsigned)reg << ")" << endl;
		i2c_start(addr, I2C_WRITE);
		mst_i2c_send(reg, true);
		i2c_continue(addr, I2C_READ);
		for(i = 0; i < len; i++) {
			mst_i2c_rec(data[i], (i == len-1) ? true : false);
		}
		wait(period_ns, SC_NS);
		i2c_active.unlock();
		time_active += sc_time_stamp() - time_trans;
		message_count++;
	}

	void i2c_listen(uint8_t &addr, bool &rw)
	{
		wait(slv_add);
		addr = dest;
		rw = rdnwr;
		return;
	}

	void slv_i2c_ack()
	{
		wait(period_ns, SC_NS);
		i2c_ack.notify();
		return;
	}

	void mst_i2c_send(uint8_t data, bool stop)
	{
		transfer = data;
		end = stop;
		data_ready.notify(SC_ZERO_TIME);
		wait(i2c_ack);
		return;
	}

	void slv_i2c_rec(uint8_t &data, bool &stop)
	{
		wait(data_ready);
		wait(8*period_ns, SC_NS);
		data = transfer;
		stop = end;
		wait(period_ns,SC_NS);
		i2c_ack.notify(SC_ZERO_TIME);
		return;
	}

	void mst_i2c_rec(uint8_t &data, bool stop)
	{
		end = stop;
		wait(data_ready);
		data = transfer;
		wait(period_ns, SC_NS);
		i2c_ack.notify(SC_ZERO_TIME);
		return;
	}

	void slv_i2c_send(uint8_t data, bool &stop)
	{
		wait(8*period_ns, SC_NS);
		transfer = data;
		data_ready.notify(SC_ZERO_TIME);
		stop = end;
		wait(i2c_ack);
		return;
	}
};

#endif
