#ifndef UAVCAN_H_
#define UAVCAN_H_

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
private:
	bool idle, sof, ack, interframe;
	unsigned char priorityHigh, byteCount, tailbyte, sourceNode, destinationNode;
	unsigned short dataType;
	unsigned long long contents;

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
};

#endif /* UAVCAN_H_ */
