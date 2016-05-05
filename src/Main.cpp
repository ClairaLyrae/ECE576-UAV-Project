#include <systemc.h>
#include <ctime>

#include "gmtl/gmtl.h"
#include "PWM.h"
#include "I2C.h"
#include "Processor.h"
#include "flightcontrol/FlightController.h"
#include "physics/Physics.h"
#include "physics/PhysicsModules.h"
#include "physics/PhysicsForces.h"

using namespace gmtl;

#define PHYSICS_SIM_TIME 20
#define PHYSICS_STEP_MS 1

// DJI Phantom 3/4 with 2312 motors (960kv) and 9.4x4.5 props
#define UAV_DRAG_COEFF 2.0
#define UAV_MASS 	1.28
#define UAV_LENGTH 	0.248
#define UAV_WIDTH 	0.248
#define UAV_HEIGHT 	0.2
#define UAV_PROP_DIAMETER 0.23876
#define UAV_PROP_PITCH 0.1143
#define UAV_MOTOR_RPM 11592
#define UAV_MOTOR_TORQUE 0.181232337

#define I2C_CLK_FREQ 400000
#define PWM_CLK_FREQ 1000

// Top Module
class Top : public sc_module
{
public:
	FlightController* hardware;
	Processor* proc;

	PhysicsSim* phys;
	Motor* motor_1;
	Motor* motor_2;
	Motor* motor_3;
	Motor* motor_4;
	PhysicsObject* uav;
	Aerodynamics* aeromodel;
	PWM_bus* pwm_bus_1;
	PWM_bus* pwm_bus_2;
	PWM_bus* pwm_bus_3;
	PWM_bus* pwm_bus_4;
	i2c_bus* iic_bus;
	uav_can_bus* can_bus;

	LSM6DS3* sensor_acc_gyro;
	LIS3MDL* sensor_mag;
	LPS22HB* sensor_baro;
	A2235H* sensor_gps;

	Top(sc_module_name name) : sc_module(name)
	{
		// CAN Bus
		can_bus = new uav_can_bus("CAN_BUS");

		// I2C Bus
		iic_bus = new i2c_bus("I2C_BUS", I2C_CLK_FREQ);
		pwm_bus_1 = new PWM_bus("PWM_BUS_1", PWM_CLK_FREQ);
		pwm_bus_2 = new PWM_bus("PWM_BUS_2", PWM_CLK_FREQ);
		pwm_bus_3 = new PWM_bus("PWM_BUS_3", PWM_CLK_FREQ);
		pwm_bus_4 = new PWM_bus("PWM_BUS_4", PWM_CLK_FREQ);

		// Processor Module
		proc = new Processor("PROCESSOR");
		proc->canif(*can_bus);

		// Sensors
		sensor_acc_gyro = new LSM6DS3("SENSOR_ACC_GYRO");
		sensor_acc_gyro->i2c_slv(*iic_bus);
		sensor_mag = new LIS3MDL("SENSOR_MAG");
		sensor_mag->i2c_slv(*iic_bus);
		sensor_baro = new LPS22HB("SENSOR_BARO");
		sensor_baro->i2c_slv(*iic_bus);
		sensor_gps = new A2235H("SENSOR_GPS");
		sensor_gps->canif(*can_bus);

		// UAV Physics Model
		uav = new PhysicsObject(UAV_MASS, UAV_LENGTH, UAV_WIDTH, UAV_HEIGHT);
		motor_1 = new Motor("MOTOR_1", Vec3d(UAV_LENGTH/2, UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM, UAV_MOTOR_TORQUE, true);
		motor_2 = new Motor("MOTOR_2", Vec3d(UAV_LENGTH/2, -UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM, UAV_MOTOR_TORQUE,false);
		motor_3 = new Motor("MOTOR_3", Vec3d(-UAV_LENGTH/2, -UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM, UAV_MOTOR_TORQUE,true);
		motor_4 = new Motor("MOTOR_4", Vec3d(-UAV_LENGTH/2, UAV_LENGTH/2, 0), UAV_PROP_DIAMETER, UAV_PROP_PITCH, UAV_MOTOR_RPM, UAV_MOTOR_TORQUE,false);
		motor_1->pwm_in(*pwm_bus_1);
		motor_2->pwm_in(*pwm_bus_2);
		motor_3->pwm_in(*pwm_bus_3);
		motor_4->pwm_in(*pwm_bus_4);
		aeromodel = new Aerodynamics(UAV_DRAG_COEFF);
		uav->addComponent(motor_1);
		uav->addComponent(motor_2);
		uav->addComponent(motor_3);
		uav->addComponent(motor_4);
		uav->addComponent(aeromodel);
		uav->enableLog("uav_physics_log.txt");
		uav->position.set(0,0,10);
		uav->addComponent(sensor_baro);
		uav->addComponent(sensor_acc_gyro);
		uav->addComponent(sensor_mag);

		// Physics Sim
		phys = new PhysicsSim("PHYSICS_SIM", PHYSICS_STEP_MS, PHYSICS_SIM_TIME);
		phys->addObject(uav);


		// Hardware Accelerator
		hardware = new FlightController("FLIGHT_CONTROL", phys, uav);
		hardware->i2c_mst(*iic_bus);
		hardware->pwm_out_1(*pwm_bus_1);
		hardware->pwm_out_2(*pwm_bus_2);
		hardware->pwm_out_3(*pwm_bus_3);
		hardware->pwm_out_4(*pwm_bus_4);
		hardware->enableLog("uav_motor_log.txt");
		hardware->canif(*can_bus);
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
