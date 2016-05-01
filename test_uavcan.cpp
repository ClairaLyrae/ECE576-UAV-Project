#include <systemc.h>
#include <UAVCAN.h>

class noisy : public sc_module
{
public:
	sc_port<uav_can_if> canif;
	
	SC_HAS_PROCESS(noisy);

	noisy(sc_module_name name) : sc_module(name)
	{
		SC_THREAD(main);
	}
	
	void main()
	{
		node = 1;
		while(!canif->can_transmit(20))
		{
			target = canif->can_listen(msgType, message, transfer, source);
			if(target == 0)
			{
				cout << "Noisy Detected Message Frame! From: " << (int)source << endl;
				cout << "Type: " << msgType << ", Payload: " << message << endl;
			}
			else if(target == node)
			{
				cout << "Noisy Detected Service Frame! From: " << (int)source << endl;
				cout << "Type: " << msgType << ", Payload: " << message << endl;
			}
		}
		if(!canif->can_message(2,100,1,0,node))
		{
			cout << "Error: Noisy wasn't heard!" << endl;
		}
		while(!canif->can_transmit(0))
		{
			target = canif->can_listen(msgType, message, transfer, source);
			if(target == 0)
			{
				cout << "Noisy Detected Message Frame! From: " << (int)source << endl;
				cout << "Type: " << msgType << ", Payload: " << message << endl;
			}
			else if(target == node)
			{
				cout << "Noisy Detected Service Frame! From: " << (int)source << endl;
				cout << "Type: " << msgType << ", Payload: " << message << endl;
			}
		}
		if(!canif->can_service(3,150,1,0,node,3))
		{
			cout << "Error: Noisy wasn't heard!" << endl;
		}
		sc_stop();
		return;
		
	}
	
private:
	unsigned short msgType;
	unsigned long long message;
	unsigned char transfer, source, target, node; 
};

class bold : public sc_module
{
public:
	sc_port<uav_can_if> canif;
	
	SC_HAS_PROCESS(bold);

	bold(sc_module_name name) : sc_module(name)
	{
		SC_THREAD(main);
	}
	
	void main()
	{
		node = 2;
		i = 0;
		while(1)
		{
			while(!canif->can_transmit(10))
			{
				target = canif->can_listen(msgType, message, transfer, source);
				if(target == 0)
				{
					cout << "Bold Detected Message Frame! From: " << (int)source << endl;
					cout << "Type: " << msgType << ", Payload: " << message << endl;
				}
				else if(target == node)
				{
					cout << "Bold Detected Service Frame! From: " << (int)source << endl;
					cout << "Type: " << msgType << ", Payload: " << message << endl;
				}
			}
			if(!canif->can_message(1,50,i++,0,node))
			{
				cout << "Error: Bold wasn't heard!" << endl;
			}
			wait(10*CAN_SPEED,SC_NS);
		}
	}

private:
	unsigned i;
	unsigned short msgType;
	unsigned long long message;
	unsigned char transfer, source, target, node; 
};

class quiet : public sc_module
{
public:
	sc_port<uav_can_if> canif;

	SC_HAS_PROCESS(quiet);

	quiet(sc_module_name name) : sc_module(name)
	{
		SC_THREAD(main);
	}
	
	void main()
	{
		node = 3;
		while(1)
		{
			target = canif->can_listen(msgType, message, transfer, source);
			if(target == 0)
			{
				cout << "Quiet Detected Message Frame! From: " << (int)source << endl;
				cout << "Type: " << msgType << ", Payload: " << message << endl;
			}
			else if(target == node)
			{
				cout << "Quiet Detected Service Frame! From: " << (int)source << endl;
				cout << "Type: " << msgType << ", Payload: " << message << endl;
			}
		}	
	}
	
private:
	unsigned short msgType;
	unsigned long long message;
	unsigned char transfer, source, target, node; 
};

int sc_main(int argc, char* argv[])
{
	uav_can_bus *can_inst;
	noisy *module_a;
	bold *module_b;
	quiet *module_c;
	
	can_inst = new uav_can_bus("UAVCAN");
	module_a = new noisy("A");
	module_b = new bold("B");
	module_c = new quiet("C");
	module_a->canif(*can_inst);
	module_b->canif(*can_inst);
	module_c->canif(*can_inst);
	
	sc_start();
	return 0;
}

