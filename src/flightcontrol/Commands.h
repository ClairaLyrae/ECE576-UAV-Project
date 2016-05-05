#ifndef COMMANDS_H_
#define COMMANDS_H_

//#include "../util/half.h"

class Command {
public:
	bool enableAttPID;
	bool enablePosPID;
	bool enableLatPosPID;
	bool enableRollPitchCmd;
	unsigned dataType;
	float posCmd[3];
	double velCmd[3];
	float attCmd[3];
	double rateCmd[3];

	void interpret(uint8_t* buff) {
		unsigned i;
		uint16_t raw;
		uint8_t header = buff[0];
		enableAttPID = header & 0x01;
		enablePosPID = header & 0x02;
		enableLatPosPID = header & 0x04;
		enableRollPitchCmd = header & 0x08;
		dataType = header & 0xF0;
		cout << sc_time_stamp() << ": Att: " << enableAttPID << " Pos: " << enablePosPID << " Lat: "  << enableLatPosPID << " Roll: "  << enableRollPitchCmd << endl;
		switch(dataType)
		{
		case 0:
			for(i = 0; i < 3; i++) {
				raw = (buff[2*i + 2] << 8) + buff[2*i + 1];
				posCmd[i] = (float)raw;

			}
			cout << sc_time_stamp() << ": Oh look, position values. " << posCmd[0] << " " << posCmd[1] << " " << posCmd[2]<< endl;
			break;
		case 16:
			for(i = 0; i < 3; i++) {
				raw = (buff[2*i + 2] << 8) + buff[2*i + 1];
				velCmd[i] = (double)raw;
			}
			break;
		case 32:
			for(i = 0; i < 3; i++) {
				raw = (buff[2*i + 2] << 8) + buff[2*i + 1];
				attCmd[i] = (float)raw;
			}
			cout << sc_time_stamp() << ": Oh look, attitude values. " << attCmd[0] << " " << attCmd[1] << " " << attCmd[2] << endl;
			break;
		case 48:
			for(i = 0; i < 3; i++) {
				raw = (buff[2*i + 2] << 8) + buff[2*i + 1];
				rateCmd[i] = (double)raw;
			}
			break;
		default:
			cout << "What did you even send?" << endl;
			break;
		}
	}
};

#endif
