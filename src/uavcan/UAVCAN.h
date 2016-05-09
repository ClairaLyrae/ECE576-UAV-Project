#ifndef UAVCAN_H_
#define UAVCAN_H_

#include <systemc.h>
#include <stdint.h>
#include "../util/util.h"

/*
Priorities range from 0 - 31. Lower is higher priority.
Node ID's range from 1 - 127.
Payload cannot exceed 7 bytes.
Transfer ID 5 bits.
*/

class uav_can_msg
{
private:
	unsigned paylength;
public:
	uint8_t payload[7];
	unsigned destID, sourceID, msgID, msgSeq;

	uav_can_msg(unsigned msgID, unsigned msgSeq, unsigned sourceID, unsigned destID) {
		set(msgID, sourceID, msgSeq, destID);
	}

	uav_can_msg() : uav_can_msg(0, 0, 0, 0) {}

	void set(unsigned msgID, unsigned msgSeq, unsigned sourceID, unsigned destID) {
		this->msgID = msgID;
		this->sourceID = sourceID;
		this->destID = destID;
		this->msgSeq = msgSeq;
	}

	bool isBroadcast() {
		return destID == 0;
	}

	unsigned length() {
		return paylength;
	}

	void packByteFloat16(uint8_t head, float fa, float fb, float fc) {
		uint16_t half;
		payload[0] = head;
		half = floatToHalf(fa);
		payload[1] = half & 0xFF;
		payload[2] = (half >> 8) & 0xFF;
		half = floatToHalf(fb);
		payload[3] = half & 0xFF;
		payload[4] = (half >> 8) & 0xFF;
		half = floatToHalf(fc);
		payload[5] = half & 0xFF;
		payload[6] = (half >> 8) & 0xFF;
		paylength = 7;
	}

	void unpackByteFloat16(uint8_t &head, float &fa, float &fb, float &fc) {
		uint16_t raw;
		head = payload[0];
		raw = (payload[2] << 8) + payload[1];
		fa = halfToFloat(raw);
		raw = (payload[4] << 8) + payload[3];
		fb = halfToFloat(raw);
		raw = (payload[6] << 8) + payload[5];
		fc = halfToFloat(raw);
	}

	void packFloat16(float fa, float fb, float fc) {
		uint16_t half;
		half = floatToHalf(fa);
		payload[0] = half & 0xFF;
		payload[1] = (half >> 8) & 0xFF;
		half = floatToHalf(fb);
		payload[2] = half & 0xFF;
		payload[3] = (half >> 8) & 0xFF;
		half = floatToHalf(fc);
		payload[4] = half & 0xFF;
		payload[5] = (half >> 8) & 0xFF;
		paylength = 6;
	}

	void unpackFloat16(float &fa, float &fb, float &fc) {
		uint16_t raw;
		raw = (payload[1] << 8) + payload[0];
		fa = halfToFloat(raw);
		raw = (payload[3] << 8) + payload[2];
		fb = halfToFloat(raw);
		raw = (payload[5] << 8) + payload[4];
		fc = halfToFloat(raw);
	}

	void packFloat32(float d) {
		uint32_t temp = *((uint32_t*)&d);
		payload[0] = temp & 0x0F;
		payload[1] = (temp >> 8) & 0x0F;
		payload[2] = (temp >> 16) & 0x0F;
		payload[3] = (temp >> 24) & 0x0F;
		paylength = 4;
	}

	void unpackFloat32(float &d) {
		uint32_t temp = (payload[3] << 24) + (payload[2] << 16) + (payload[1] << 8) + payload[0];
		d = *((float*)&temp);
	}
};

class uav_can_if : virtual public sc_interface
{
public:
	virtual unsigned can_listen(uav_can_msg &msg) = 0;
	virtual bool can_transmit(unsigned char priority) = 0;
	virtual bool can_message(uav_can_msg msg) = 0;
};

class uav_can_bus : public sc_module, public uav_can_if
{
private:
	bool idle, ack, interframe, display;
	unsigned char priorityHigh;
	double period_ns;
	uav_can_msg currentMessage;

public:
	SC_HAS_PROCESS(uav_can_bus);

	uav_can_bus(sc_module_name name, double freq) : sc_module(name)
	{
		SC_THREAD(arbiter);
		setFrequency(freq);
		display = false;
	}

	void setFrequency(double freq) {
		period_ns = (1000000000.0/freq);
	}

	void showTraffic(bool b) {
		display = b;
	}

	unsigned can_listen(uav_can_msg &msg)
	{
		wait(frameReady);
		msg = currentMessage;
		ack = true;
		wait(canRelease);
		return msg.destID; //return 0 if broadcast
	}

	bool can_transmit(unsigned char priority)
	{
		if(interframe)
			wait(canIdle);
		if(idle) {
			sof.notify();
			if(priority < priorityHigh)
				priorityHigh = priority;
			wait(priorityCheck);
			if(priority == priorityHigh)
				return true;
			else
				return false; //not high enough priority
		} else {
			return false; //other frame transmission in progress
		}
	}

	bool can_message(uav_can_msg msg)
	{
		currentMessage = msg;
		frameLoaded.notify();
		wait(canRelease);
		return ack;
	}

	void arbiter()
	{
		while(true)
		{
			priorityHigh = 32;
			ack = false;
			interframe = false;
			idle = true;
			canIdle.notify();	// Notify listeners that bus is ready for transactions
			wait(sof);
			wait(period_ns,SC_NS);
			idle = false;
			wait(34*period_ns,SC_NS);	// Time for header
			priorityCheck.notify();
			wait(frameLoaded);
			if(display)
				cout << "[" << sc_time_stamp() << "] UAVCAN message (msg=" << (unsigned)currentMessage.msgID <<  ", src=" << (unsigned)currentMessage.sourceID << ", dest=" << (unsigned)currentMessage.destID << ")" << endl;
			wait(27*period_ns,SC_NS);
			wait(currentMessage.length()*8*period_ns,SC_NS);
			wait(((61+currentMessage.length()*8)/5)*period_ns,SC_NS); //stuffing estimation
			frameReady.notify();
			wait(10*period_ns,SC_NS);
			interframe = true;
			canRelease.notify();
			wait(3*period_ns,SC_NS);
		}
	}

	sc_event canIdle, priorityCheck, frameLoaded, frameReady, canRelease, sof;
};

#endif
