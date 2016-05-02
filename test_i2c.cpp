/*
Simple implementation of i2c.
Approximate Timed.
No Clock Stretching.
Arbitration hand-waved.
*/

//change notify times in data transfer

#include <systemc.h>
#include "I2C.h"

#define IIC_SPEED 2500 // ns, 400 kHz
#define SLV_ADDR 8
#define TARGET_REG 13
#define I2C_WRITE false
#define I2C_READ true

class mst_sham : public sc_module
{
public:
	sc_port<iic_mst_if> iicif;

	SC_HAS_PROCESS(mst_sham);

	mst_sham(sc_module_name name, unsigned addr) : sc_module(name), slvaddr(addr)
	{
		SC_THREAD(main);
	}

	void main()
	{
		unsigned borrow[2];
		//single read

		cout << "Single read operation" << endl;

		iicif->iic_start(slvaddr,I2C_WRITE);
		iicif->mst_iic_write(TARGET_REG,true);
		iicif->iic_continue(slvaddr, I2C_READ);
		iicif->mst_iic_read(borrow[0],true);
		iicif->iic_stop();

		cout << "Master read from register " << TARGET_REG << " = " << borrow[0] << endl;


		cout << "\nMultiple read operation" << endl;

		//multi read
		iicif->iic_start(slvaddr, I2C_WRITE);
		iicif->mst_iic_write(TARGET_REG,true);
		iicif->iic_continue(slvaddr, I2C_READ);
		iicif->mst_iic_read(borrow[0],false);
		iicif->mst_iic_read(borrow[1],true);
		iicif->iic_stop();

		cout << "Master read from register " << TARGET_REG << " = " << borrow[0] << endl;
		cout << "Master read from register " << TARGET_REG+1 << " = " << borrow[1] << endl;



		cout << "\nSingle write operation" << endl;
		cout << "Master write to register " << TARGET_REG << " = " << 20 << endl;

		//single write
		iicif->iic_start(slvaddr, I2C_WRITE);
		iicif->mst_iic_write(TARGET_REG,false);
		iicif->mst_iic_write(20,true);
		iicif->iic_stop();


		cout << "\nMultiple write operation\n";
		cout << "Master write to register " << TARGET_REG << " = " << 40 << endl;
		cout << "Master write to register " << TARGET_REG+1 << " = " << 35 << endl;

		//multi write
		iicif->iic_start(slvaddr, I2C_WRITE);
		iicif->mst_iic_write(TARGET_REG,false);
		iicif->mst_iic_write(40,false);
		iicif->mst_iic_write(35,true);
		iicif->iic_stop();

		sc_stop();
		return;
	}

private:
	unsigned slvaddr;
};

class slv_sham : public sc_module
{
public:
	sc_port<iic_slv_if> iicif;

	SC_HAS_PROCESS(slv_sham);

	slv_sham(sc_module_name name, unsigned addr) : sc_module(name), slvaddr(addr)
	{
		SC_THREAD(main);
	}
	void main()
	{
		unsigned registers[20];
		unsigned req_addr, reg_loc, i;
		bool stop;
		bool rw;
		registers[13] = 3;
		registers[14] = 5;
		while(1)
		{
			i = 0;
			req_addr = 0;
			stop = false;
			iicif->iic_listen(req_addr, rw);
			if(req_addr == slvaddr) {
				if(!rw)
				{
					iicif->slv_iic_ack();
					iicif->slv_iic_rec(reg_loc,stop);
					while(!stop)
					{
						iicif->slv_iic_rec(registers[reg_loc + i],stop);
						i++;
					}
					cout << "Slave Register 13 = " << registers[13] << endl;
					cout << "Slave Register 14 = " << registers[14] << endl;
				}
				else
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
	}

private:
	unsigned slvaddr;
};

int sc_main(int argc, char* argv[])
{
	iic_bus *iic_inst;
	slv_sham *test_slave;
	mst_sham *test_master;

	iic_inst = new iic_bus("EyeTwoCee", 400000);
	test_slave = new slv_sham("Slave",SLV_ADDR);
	test_master = new mst_sham("Master",SLV_ADDR);
	test_slave->iicif(*iic_inst);
	test_master->iicif(*iic_inst);

	sc_start();
	return 0;
}
