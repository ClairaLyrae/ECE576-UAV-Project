#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

class Sensors {
public:
	double temp;
	double pressure;
	double gyro[3];
	double accel[3];
	double mag[3];
	float gps_longitude;
	float gps_latitude;
	float gps_altitude;
	float gps_heading;
	float gps_velocity;
};

#endif
