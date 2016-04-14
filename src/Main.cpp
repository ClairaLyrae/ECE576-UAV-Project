#include <systemc.h>
#include <ctime>

#include "gmtl/gmtl.h"
#include "Processor.h"
#include "HardwareAccel.h"
#include "physics/Physics.h"
#include "physics/PhysicsModules.h"
#include "physics/PhysicsForces.h"

using namespace gmtl;

#define PHYSICS_SIM_TIME 100
#define PHYSICS_STEP_MS 10

// DJI Phantom 3/4 with 2312 motors (960kv) and 9.4x4.5 props
#define UAV_DRAG_COEFF 2.0
#define UAV_MASS 	1.28
#define UAV_LENGTH 	0.248
#define UAV_WIDTH 	0.248
#define UAV_HEIGHT 	0.2
#define UAV_PROP_DIAMETER 0.23876
#define UAV_PROP_PITCH 0.1143
#define UAV_MOTOR_RPM 11592

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
	Aerodynamics* aeromodel;

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

		// UAV Physics Model
		uav = new PhysicsObject(UAV_MASS, UAV_LENGTH, UAV_WIDTH, UAV_HEIGHT);
		motor_1 = new Motor("MOTOR_1", Vec3d(UAV_LENGTH/2, UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM);
		motor_2 = new Motor("MOTOR_2", Vec3d(UAV_LENGTH/2, -UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM);
		motor_3 = new Motor("MOTOR_3", Vec3d(-UAV_LENGTH/2, -UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM);
		motor_4 = new Motor("MOTOR_4", Vec3d(-UAV_LENGTH/2, UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM);
		aeromodel = new Aerodynamics(UAV_DRAG_COEFF);
		uav->addComponent(motor_1);
		uav->addComponent(motor_2);
		uav->addComponent(motor_3);
		uav->addComponent(motor_4);
		uav->addComponent(aeromodel);
		uav->enableLog("uav_physics_log.txt");
		motor_1->thrustLevel = 0.98;
		motor_2->thrustLevel = 1.0;
		motor_3->thrustLevel = 0.98;
		motor_4->thrustLevel = 0.98;

		// Physics Sim
		phys = new PhysicsSim("PHYSICS_SIM", PHYSICS_STEP_MS, PHYSICS_SIM_TIME);
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
