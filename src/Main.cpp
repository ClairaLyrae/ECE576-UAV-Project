#include <systemc.h>
#include <ctime>

#include "gmtl.h"
#include "Sensors.h"
#include "Physics.h"
#include "Processor.h"
#include "HardwareAccel.h"
#include "sensors/Barometer.h"

using namespace gmtl;

// Top Module
class Top : public sc_module
{
public:
	HardwareAccel* hardware;
	Processor* proc;

	Physics* phys;
	Motor* motor_1;
	Motor* motor_2;
	Motor* motor_3;
	Motor* motor_4;
	PhysicsObject* uav;
	Gravity* grav;
	Drag* drag;

	Accelerometer* sensor_acc;
	Magnetometer* sensor_mag;
	Barometer* sensor_baro;
	GPS* sensor_gps;
	Gyroscope* sensor_gyro;

	Top(sc_module_name name) : sc_module(name)
	{
		// Processor Module
		proc = new Processor("PROCESSOR");

		// Hardware
		hardware = new HardwareAccel("HARDWARE_ACCEL");

		// Sensors
		sensor_acc = new Accelerometer("SENSOR_ACC");
		sensor_mag = new Magnetometer("SENSOR_MAG");
		sensor_baro = new Barometer("SENSOR_BARO");
		sensor_gps = new GPS("SENSOR_GPS");
		sensor_gyro = new Gyroscope("SENSOR_GYRO");

		// Physics Module
		phys = new Physics("PHYSICS");
		uav = new PhysicsObject(10.0);
		motor_1 = new Motor(Vec3d(1, 1, 0));
		motor_2 = new Motor(Vec3d(1, -1, 0));
		motor_3 = new Motor(Vec3d(-1, -1, 0));
		motor_4 = new Motor(Vec3d(-1, 1, 0));
		grav = new Gravity(Vec3d(0, 0, -9.8));
		drag = new Drag(1, 1, 1);
		uav->addComponent(*motor_1);
		uav->addComponent(*motor_2);
		uav->addComponent(*motor_3);
		uav->addComponent(*motor_4);
		uav->addComponent(*grav);
		uav->addComponent(*drag);
		phys->addObject(*uav);
	}
};

// System C Main
int sc_main(int argc, char *argv[]) {

	// Instantiate modules
	Top top_inst("TOP");

	// Start simulation
	cout << "Beginning simulation...\n" << endl;
	sc_start();
	cout << "\nSimulation stopped." << endl;

	return 0;
}
