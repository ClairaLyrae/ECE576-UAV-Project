#include <systemc.h>
#include <ctime>

#include "gmtl/gmtl.h"
#include "Sensors.h"
#include "Processor.h"
#include "HardwareAccel.h"
#include "Physics.h"
#include "Actuators.h"

using namespace gmtl;

#define PHYSICS_SIM_TIME 100
#define PHYSICS_STEP_MS 10
#define PHYSICS_GRAVITY -9.80665
#define PHYSICS_DRAG_COEFF 0.4

#define UAV_MASS 	10
#define UAV_LENGTH 	0.5
#define UAV_WIDTH 	0.5
#define UAV_HEIGHT 	0.1

// Top Module
class Top : public sc_module
{
public:
	HardwareAccel* hardware;
	Processor* proc;

	PhysicsSim* phys;
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
		phys = new PhysicsSim("PHYSICS_SIM", PHYSICS_STEP_MS, PHYSICS_SIM_TIME);
		uav = new PhysicsObject(UAV_MASS, UAV_LENGTH, UAV_WIDTH, UAV_HEIGHT);
		motor_1 = new Motor("MOTOR_1", Vec3d(1, 1, 0));
		motor_2 = new Motor("MOTOR_2", Vec3d(1, -1, 0));
		motor_3 = new Motor("MOTOR_3", Vec3d(-1, -1, 0));
		motor_4 = new Motor("MOTOR_4", Vec3d(-1, 1, 0));
		grav = new Gravity(Vec3d(0, 0, PHYSICS_GRAVITY));
		drag = new Drag(1, PHYSICS_DRAG_COEFF, 1);
		uav->addComponent(motor_1);
		uav->addComponent(motor_2);
		uav->addComponent(motor_3);
		uav->addComponent(motor_4);
		uav->addComponent(grav);
		//uav->addComponent(drag);
		uav->enableLog("uav.txt");
		phys->addObject(uav);
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
