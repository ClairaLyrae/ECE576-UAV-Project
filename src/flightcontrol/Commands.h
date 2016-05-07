#ifndef COMMANDS_H_
#define COMMANDS_H_

//#include "../util/half.h"

class Command {
public:
	bool enableAttPID;
	bool enableAltPID;
	bool enableLatPID;
	unsigned dataType;
	float posCmd[3];
	double velCmd[3];
	float attCmd[3];
	double rateCmd[3];
};

#endif
