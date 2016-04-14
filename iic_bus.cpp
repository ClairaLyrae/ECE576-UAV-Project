/*
Simple implementation of i2c.
Approximate Timed.
No Clock Stretching.
Arbitration hand-waved.
*/

//change notify times in data transfer

#include <systemc.h>

#define IIC_SPEED 2500 // ns, 400 kHz
#define SLV_W 8
#define SLV_R 9
#define TARGET_REG 13

class iic_mst_if : virtual public sc_interface
{
public:
		virtual void iic_start(unsigned addr) = 0;
		virtual void mst_iic_write(unsigned data, bool stop) = 0;
		virtual void mst_iic_read(unsigned &data, bool stop) = 0;
		virtual void iic_continue(unsigned addr) = 0;
		virtual void iic_stop() = 0;
};

class iic_slv_if : virtual public sc_interface
{
public:
		virtual void iic_listen(unsigned &addr) = 0;
		virtual void slv_iic_ack() = 0;
		virtual void slv_iic_rec(unsigned &data, bool &stop) = 0;
		virtual void slv_iic_send(unsigned data, bool &stop) = 0;
};

class iic_bus : public sc_module, public iic_mst_if, public iic_slv_if
{
public:

	SC_HAS_PROCESS(iic_bus);
	
	iic_bus(sc_module_name name) : sc_module(name)
	{
		
	}

	sc_event iic_ack, data_ready, slv_add, slv_ready;
	unsigned transfer, dest, end;
	sc_mutex iic_active;
	
	void iic_start(unsigned addr)
	{
		iic_active.lock();
		dest = addr;
		wait(9*IIC_SPEED,SC_NS);
		slv_add.notify();
		wait(iic_ack);
		return;
	}
	
	void iic_continue(unsigned addr)
	{
		dest = addr;
		wait(9*IIC_SPEED,SC_NS);
		slv_add.notify();
		wait(iic_ack);
		return;
	}
	
	void iic_listen(unsigned &addr)
	{
		wait(slv_add);
		addr = dest;
		return;
	}
	
	void slv_iic_ack()
	{
		wait(IIC_SPEED,SC_NS);
		iic_ack.notify();
		return;
	}
	
	void iic_stop()
	{
		wait(IIC_SPEED,SC_NS);
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
		wait(8*IIC_SPEED,SC_NS);
		data = transfer;
		stop = end;
		wait(IIC_SPEED,SC_NS);
		iic_ack.notify(SC_ZERO_TIME);
		return;
	}
	
	void mst_iic_read(unsigned &data, bool stop)
	{
		end = stop;
		wait(data_ready);
		data = transfer;
		wait(IIC_SPEED,SC_NS);
		iic_ack.notify(SC_ZERO_TIME);
		return;
	}
	
	void slv_iic_send(unsigned data, bool &stop)
	{
		wait(8*IIC_SPEED,SC_NS);
		transfer = data;
		data_ready.notify(SC_ZERO_TIME);
		stop = end;
		wait(iic_ack);
		return;
	}
};

class mst_sham : public sc_module
{
public:
	sc_port<iic_mst_if> iicif;
	
	SC_HAS_PROCESS(mst_sham);
	
	mst_sham(sc_module_name name, unsigned target_r, unsigned target_w) : sc_module(name), w_target(target_w), r_target(target_r)
	{
		SC_THREAD(main);
	}
	
	void main()
	{
		unsigned borrow[2];
		//single read
		iicif->iic_start(w_target);
		iicif->mst_iic_write(TARGET_REG,true);
		iicif->iic_continue(r_target);
		iicif->mst_iic_read(borrow[0],true);
		cout << "read single: " << borrow[0] << endl;
		iicif->iic_stop();
		//multi read
		iicif->iic_start(w_target);
		iicif->mst_iic_write(TARGET_REG,true);
		iicif->iic_continue(r_target);
		iicif->mst_iic_read(borrow[0],false);
		iicif->mst_iic_read(borrow[1],true);
		iicif->iic_stop();
		cout << "read 1: " << borrow[0] << endl;
		cout << "read 2: " << borrow[1] << endl;
		//single write
		iicif->iic_start(w_target);
		iicif->mst_iic_write(TARGET_REG,false);
		iicif->mst_iic_write(20,true);
		iicif->iic_stop();
		//multi write
		iicif->iic_start(w_target);
		iicif->mst_iic_write(TARGET_REG,false);
		iicif->mst_iic_write(40,false);
		iicif->mst_iic_write(35,true);
		iicif->iic_stop();

		sc_stop();
		return;
	}

private:
	unsigned w_target;
	unsigned r_target;
};

class slv_sham : public sc_module
{
public:
	sc_port<iic_slv_if> iicif;
	
	SC_HAS_PROCESS(slv_sham);

	slv_sham(sc_module_name name, unsigned addr_r, unsigned addr_w) : sc_module(name), w_addr(addr_w), r_addr(addr_r)
	{
		SC_THREAD(main);
	}
	void main()
	{
		unsigned registers[20];
		unsigned req_addr, reg_loc, i;
		bool stop;
		registers[13] = 3;
		registers[14] = 5;
		while(1)
		{
			i = 0;
			req_addr = 0;
			stop = false;
			iicif->iic_listen(req_addr);
			if(req_addr == w_addr)
			{
				iicif->slv_iic_ack();
				iicif->slv_iic_rec(reg_loc,stop);
				while(!stop)
				{
					iicif->slv_iic_rec(registers[reg_loc + i],stop);
					i++;
				}
				cout << "13: " << registers[13] << endl;
				cout << "14: " << registers[14] << endl;
			}
			else if(req_addr == r_addr)
			{
				iicif->slv_iic_ack();
				while(!stop)
				{
					iicif->slv_iic_send(registers[reg_loc + i],stop);
					i++;
				}
			}
		}
	}
	
private:
	unsigned w_addr;
	unsigned r_addr;
};

int sc_main(int argc, char* argv[])
{
	iic_bus *iic_inst;
	slv_sham *test_slave;
	mst_sham *test_master;
	
	iic_inst = new iic_bus("EyeTwoCee");
	test_slave = new slv_sham("Slave",SLV_R,SLV_W);
	test_master = new mst_sham("Master",SLV_R,SLV_W);
	test_slave->iicif(*iic_inst);
	test_master->iicif(*iic_inst);
	
	sc_start();
	return 0;
}