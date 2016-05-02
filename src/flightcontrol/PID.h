#ifndef PID_H_
#define PID_H_

class PID {
private:
public:
	double integratorState;
	double filterState;
	double P, I, D, Limit;
	double state;

	PID() : PID(0, 0, 0, 0, 0, 0) {}

	PID(double p, double i, double d, double lim, double is, double fs) {
		initialize(p, i, d, lim, is, fs);
	}

	void initialize(double p, double i, double d, double lim, double is, double fs) {
		integratorState = is;
		filterState = fs;
		P = p;
		I = i;
		D = d;
		Limit = lim;
		state = 0;
	}

	double update(double error, double deltaT) {
	    double dTerm;
	    double pidSum;
	    double pidLimited;

	    dTerm = ((error*D) - filterState) * 100.0f;
	    pidSum = (error*P) + integratorState + dTerm;

	    if (pidSum > Limit) {
	        pidLimited = Limit;
	    } else {
	        pidLimited = -Limit;
	        if (!(pidSum < (-Limit))) {
	            pidLimited = pidSum;
	        }
	    }
	    integratorState += ((error * I) + 100.0f * (pidLimited - pidSum)) * deltaT;
	    filterState += deltaT * dTerm;
	    state = pidLimited;
	    return pidLimited;
	}

	void setPIDstate(double value) {
	    integratorState = value;
	    filterState     = value;
	}

	void zeroPIDstate() {
		setPIDstate(0.0f);
	}
};

#endif
