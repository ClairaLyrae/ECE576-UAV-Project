#ifndef I2C_H_
#define I2C_H_

#include <systemc.h>

class iic_mst_if : virtual public sc_interface
{
public:
	virtual void iic_start(unsigned addr, bool rw) = 0;
	virtual void mst_iic_write(unsigned data, bool stop) = 0;
	virtual void mst_iic_read(unsigned &data, bool stop) = 0;
	virtual void iic_continue(unsigned addr, bool rw) = 0;
	virtual void iic_stop() = 0;
};

class iic_slv_if : virtual public sc_interface
{
public:
	virtual void iic_listen(unsigned &addr, bool &rw) = 0;
	virtual void slv_iic_ack() = 0;
	virtual void slv_iic_rec(unsigned &data, bool &stop) = 0;
	virtual void slv_iic_send(unsigned data, bool &stop) = 0;
};

class iic_bus : public sc_module, public iic_mst_if, public iic_slv_if
{
private:
	sc_event iic_ack, data_ready, slv_add, slv_ready;
	unsigned transfer, dest, end;
	bool rdnwr;
	sc_mutex iic_active;

	unsigned period_ns;
public:
	SC_HAS_PROCESS(iic_bus);

	iic_bus(sc_module_name name, double freq) : sc_module(name)
	{
		period_ns = 1000000000.0/freq;
		transfer = 0;
		dest = 0;
		end = 0;
		rdnwr = false;
	}

	void iic_start(unsigned addr, bool rw)
	{
		iic_active.lock();
		dest = addr;
		rdnwr = rw;
		wait(9*period_ns,SC_NS);
		slv_add.notify();
		wait(iic_ack);
		return;
	}

	void iic_continue(unsigned addr, bool rw)
	{
		dest = addr;
		rdnwr = rw;
		wait(9*period_ns,SC_NS);
		slv_add.notify();
		wait(iic_ack);
		return;
	}

	void iic_listen(unsigned &addr, bool &rw)
	{
		wait(slv_add);
		addr = dest;
		rw = rdnwr;
		return;
	}

	void slv_iic_ack()
	{
		wait(period_ns,SC_NS);
		iic_ack.notify();
		return;
	}

	void iic_stop()
	{
		wait(period_ns,SC_NS);
		iic_active.unlock();
		return;
	}

	void mst_iic_write(unsigned data, bool stop)
	{
		transfer = data;
		end = stop;
		data_ready.notify(SC_ZERO_TIME);
		wait(iic_ack);
		return;
	}

	void slv_iic_rec(unsigned &data, bool &stop)
	{
		wait(data_ready);
		wait(8*period_ns,SC_NS);
		data = transfer;
		stop = end;
		wait(period_ns,SC_NS);
		iic_ack.notify(SC_ZERO_TIME);
		return;
	}

	void mst_iic_read(unsigned &data, bool stop)
	{
		end = stop;
		wait(data_ready);
		data = transfer;
		wait(period_ns,SC_NS);
		iic_ack.notify(SC_ZERO_TIME);
		return;
	}

	void slv_iic_send(unsigned data, bool &stop)
	{
		wait(8*period_ns,SC_NS);
		transfer = data;
		data_ready.notify(SC_ZERO_TIME);
		stop = end;
		wait(iic_ack);
		return;
	}
};

#endif
