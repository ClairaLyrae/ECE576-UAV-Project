#include <systemc.h>

#define CAN_SPEED 1000 //ns

/*
Priorities range from 0 - 31. Lower is higher priority.
Node ID's range from 1 - 127.
Payload cannot exceed 7 bytes.
Transfer ID 5 bits.
*/

class uav_can_if : virtual public sc_interface
{
public:
	virtual unsigned char can_listen(unsigned short &msgID, unsigned long long &payload, unsigned char &transferID, unsigned char &sourceID) = 0;
	virtual bool can_transmit(unsigned char priority) = 0;
	virtual bool can_message(unsigned short msgID, unsigned long long payload, unsigned char paylength, unsigned char transferID, unsigned char sourceID) = 0;
	virtual bool can_service(unsigned short msgID, unsigned long long payload, unsigned char paylength, unsigned char transferID, unsigned char sourceID, unsigned char destinationID) = 0;
};

class uav_can_bus : public sc_module, public uav_can_if
{
public:
	
	SC_HAS_PROCESS(uav_can_bus);
	
	uav_can_bus(sc_module_name name) : sc_module(name)
	{
		SC_THREAD(arbiter);
	}
	
	unsigned char can_listen(unsigned short &msgID, unsigned long long &payload, unsigned char &transferID, unsigned char &sourceID)
	{
		wait(frameReady);
		msgID = dataType;
		payload = contents;
		transferID = tailbyte;
		sourceID = sourceNode;
		ack = true;
		wait(canRelease);
		return destinationNode; //return 0 if broadcast
	}
	
	bool can_transmit(unsigned char priority)
	{
		if(interframe)
		{
			wait(canIdle);	
		}
		if(idle)
		{
			sof = true;
			if(priority < priorityHigh)
			{
				priorityHigh = priority;
			}
			wait(priorityCheck);
			if(priority == priorityHigh)
			{
				return true;
			}
			else
			{
				return false; //not high enough priority
			}
		}
		else
		{
			return false; //other frame transmission in progress
		}
	}
	
	bool can_message(unsigned short msgID, unsigned long long payload, unsigned char paylength, unsigned char transferID, unsigned char sourceID)
	{
		dataType = msgID;
		contents = payload;
		byteCount = paylength;
		tailbyte = transferID;
		sourceNode = sourceID;
		frameLoaded.notify();
		wait(canRelease);
		return ack;	
	}
	
	bool can_service(unsigned short msgID, unsigned long long payload, unsigned char paylength, unsigned char transferID, unsigned char sourceID, unsigned char destinationID)
	{
		dataType = msgID;
		contents = payload;
		byteCount = paylength;
		tailbyte = transferID;
		sourceNode = sourceID;
		destinationNode = destinationID;
		frameLoaded.notify();
		wait(canRelease);
		return ack;	
	}
	
	void arbiter()
	{
		while(1)
		{
			destinationNode = 0;
			priorityHigh = 32;
			sof = false;
			ack = false;
			interframe = false;
			idle = true;
			canIdle.notify();
			do{
				wait(CAN_SPEED,SC_NS);
			}while(!sof);
			idle = false;
			wait(34*CAN_SPEED,SC_NS);
			priorityCheck.notify();
			wait(frameLoaded);
			wait(28*CAN_SPEED,SC_NS);
			wait(byteCount*8*CAN_SPEED,SC_NS);
			frameReady.notify();
			wait(((62+byteCount*8)/5)*CAN_SPEED,SC_NS); //stuffing estimation
			wait(9*CAN_SPEED,SC_NS);
			interframe = true;
			canRelease.notify();
			wait(3*CAN_SPEED,SC_NS);
		}
	}
	
	sc_event canIdle, priorityCheck, frameLoaded, frameReady, canRelease;
	
private:
	bool idle, sof, ack, interframe;
	unsigned char priorityHigh, byteCount, tailbyte, sourceNode, destinationNode;
	unsigned short dataType;
	unsigned long long contents;
};

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

