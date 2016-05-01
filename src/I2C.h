#ifndef I2C_H_
#define I2C_H_

#include <systemc.h>

#define I2C_WRITE false
#define I2C_READ true

class i2c_mst_if : virtual public sc_interface
{
public:
	virtual void i2c_start(unsigned addr, bool rw) = 0;
	virtual void mst_i2c_write(unsigned data, bool stop) = 0;
	virtual void mst_i2c_read(unsigned &data, bool stop) = 0;
	virtual void i2c_continue(unsigned addr, bool rw) = 0;
	virtual void i2c_stop() = 0;
	virtual void i2c_write(unsigned addr, unsigned reg, unsigned len, unsigned *data) = 0;
	virtual void i2c_read(unsigned addr, unsigned reg, unsigned len, unsigned *data) = 0;
};

class i2c_slv_if : virtual public sc_interface
{
public:
	virtual void i2c_listen(unsigned &addr, bool &rw) = 0;
	virtual void slv_i2c_ack() = 0;
	virtual void slv_i2c_rec(unsigned &data, bool &stop) = 0;
	virtual void slv_i2c_send(unsigned data, bool &stop) = 0;
};

class i2c_bus : public sc_module, public i2c_mst_if, public i2c_slv_if
{
private:
	sc_event i2c_ack, data_ready, slv_add, slv_ready;
	unsigned transfer, dest, end;
	bool rdnwr;
	sc_mutex i2c_active;

	unsigned period_ns;
public:
	SC_HAS_PROCESS(i2c_bus);

	i2c_bus(sc_module_name name, double freq) : sc_module(name)
	{
		period_ns = 1000000000.0/freq;
		transfer = 0;
		dest = 0;
		end = 0;
		rdnwr = false;
	}

	void i2c_write(unsigned addr, unsigned reg, unsigned len, unsigned *data) {
		unsigned i = 0;
		i2c_start(addr, I2C_WRITE);
		mst_i2c_write(reg, true);
		for(i = 0; i < len; i++) {
			mst_i2c_write(data[i], (i == len-1) ? true : false);
		}
		i2c_stop();
	}

	void i2c_read(unsigned addr, unsigned reg, unsigned len, unsigned *data) {
		unsigned i = 0;
		i2c_start(addr, I2C_WRITE);
		mst_i2c_write(reg, true);
		i2c_continue(addr, I2C_READ);
		for(i = 0; i < len; i++) {
			mst_i2c_read(data[reg + i], (i == len-1) ? true : false);
		}
		i2c_stop();
	}

	void i2c_start(unsigned addr, bool rw)
	{
		i2c_active.lock();
		dest = addr;
		rdnwr = rw;
		wait(9*period_ns,SC_NS);
		slv_add.notify();
		wait(i2c_ack);
		return;
	}

	void i2c_continue(unsigned addr, bool rw)
	{
		dest = addr;
		rdnwr = rw;
		wait(9*period_ns,SC_NS);
		slv_add.notify();
		wait(i2c_ack);
		return;
	}

	void i2c_listen(unsigned &addr, bool &rw)
	{
		wait(slv_add);
		addr = dest;
		rw = rdnwr;
		return;
	}

	void slv_i2c_ack()
	{
		wait(period_ns,SC_NS);
		i2c_ack.notify();
		return;
	}

	void i2c_stop()
	{
		wait(period_ns,SC_NS);
		i2c_active.unlock();
		return;
	}

	void mst_i2c_write(unsigned data, bool stop)
	{
		transfer = data;
		end = stop;
		data_ready.notify(SC_ZERO_TIME);
		wait(i2c_ack);
		return;
	}

	void slv_i2c_rec(unsigned &data, bool &stop)
	{
		wait(data_ready);
		wait(8*period_ns,SC_NS);
		data = transfer;
		stop = end;
		wait(period_ns,SC_NS);
		i2c_ack.notify(SC_ZERO_TIME);
		return;
	}

	void mst_i2c_read(unsigned &data, bool stop)
	{
		end = stop;
		wait(data_ready);
		data = transfer;
		wait(period_ns,SC_NS);
		i2c_ack.notify(SC_ZERO_TIME);
		return;
	}

	void slv_i2c_send(unsigned data, bool &stop)
	{
		wait(8*period_ns,SC_NS);
		transfer = data;
		data_ready.notify(SC_ZERO_TIME);
		stop = end;
		wait(i2c_ack);
		return;
	}
};

#endif

//#ifndef I2C_H_
//#define I2C_H_
//
//#include <systemc.h>
//
//#define I2C_WRITE false
//#define I2C_READ true
//
//class i2c_mst_if : virtual public sc_interface
//{
//public:
//	virtual void i2c_write(unsigned addr, unsigned reg, unsigned len, unsigned *data) = 0;
//	virtual void i2c_read(unsigned addr, unsigned reg, unsigned len, unsigned *data) = 0;
//};
//
//class i2c_slv_if : virtual public sc_interface
//{
//public:
//	virtual void i2c_listen(unsigned &addr, bool &rw, unsigned &reg, unsigned* data) = 0;
//};
//
//class i2c_bus : public sc_module, public i2c_mst_if, public i2c_slv_if
//{
//private:
//	sc_event beginTransaction, registerReady, dataReady, slaveAck, masterAck, endTransaction;
//	unsigned transfer, dest, destreg, end;
//	bool rdnwr;
//	sc_mutex i2c_active;
//
//	unsigned period_ns;
//public:
//	SC_HAS_PROCESS(i2c_bus);
//
//	i2c_bus(sc_module_name name, double freq) : sc_module(name)
//	{
//		period_ns = 1000000000.0/freq;
//		transfer = 0;
//		dest = 0;
//		end = 0;
//		rdnwr = false;
//	}
//
//	void i2c_write(unsigned addr, unsigned reg, unsigned len, unsigned *data) {
//		i2c_active.lock();
//		dest = addr;
//		rdnwr = I2C_WRITE;
//		destreg = reg;
//		wait(9*period_ns, SC_NS);	// Send address
//		beginTransaction.notify();
//		wait(slaveAck);
//		wait(9*period_ns, SC_NS);	// Send register address
//		registerReady.notify(SC_ZERO_TIME);
//		wait(slaveAck);
//
//		for(unsigned i = 0; i < len; i++) {
//			wait(9*period_ns, SC_NS);	// Send data
//			transfer = data[i];
//			dataReady.notify();
//			wait(slaveAck);
//		}
//		endTransaction.notify();
//
//		i2c_active.unlock();
//	}
//
//	void i2c_read(unsigned addr, unsigned reg, unsigned len, unsigned *data) {
//		i2c_active.lock();
//		dest = addr;
//		rdnwr = I2C_READ;
//		destreg = reg;
//		end = false;
//		wait(9*period_ns, SC_NS);	// Send address
//		beginTransaction.notify();
//		cout << "Waiting for slave acknowledge on address" << endl;
//		wait(slaveAck);
//		wait(9*period_ns, SC_NS);	// Send register address
//		if(len == 0) {
//			end = true;
//			registerReady.notify(SC_ZERO_TIME);
//			cout << "Master waiting for slave acknowledge on register, end of sequence" << endl;
//		} else {
//			registerReady.notify(SC_ZERO_TIME);
//			cout << "Master waiting for slave acknowledge on register" << endl;
//			wait(slaveAck);
//
//			for(unsigned i = 0; i < len; i++) {
//				cout << "Master waiting for slave to send data #" << i+1 << endl;
//				wait(dataReady);	// Send data
//				data[i] = transfer;
//				masterAck.notify();
//			}
//			end = true;
//			endTransaction.notify();
//			cout << "Ending transaction" << endl;
//		}
//
//		i2c_active.unlock();
//	}
//
//
//	void i2c_listen(unsigned &addr, bool &rw)
//	{
//		int i = 0;
//		wait(beginTransaction);
//		addr = dest;
//	}
//
//	void i2c_slave_ack() {
//		slaveAck.notify();
//	}
//
//	void i2c_slave_rec(unsigned &reg, unsigned *data) {
//		wait(registerReady);
//		reg = destreg;
//		if(!end) {
//			slaveAck.notify();
//			if(rw == I2C_WRITE) {
//				while(true) {
//					wait(dataReady | endTransaction);	// Send data
//					if(end)
//						break;
//					data[i + 1] = transfer;
//					slaveAck.notify();
//					i++;
//				}
//			} else {
//				while(true) {
//					wait(9*period_ns, SC_NS);	// Send data
//					transfer = data[reg + i];
//					cout << "Slave data ready: " << transfer << endl;
//					dataReady.notify();
//					wait(masterAck | endTransaction);	// Send data
//					if(end)
//						break;
//					i++;
//				}
//			}
//		}
//
//		return;
//	}
//};
//
//#endif
