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
#include "util/util.h"
#include "uavcan/UAVCAN.h"
#include "uavcan/FrameTypes.h"
#include "uavcan/FrameGenerator.h"
#include "Config.h"

using namespace gmtl;
using namespace std;

// DJI Phantom 3/4 with 2312 motors (960kv) and 9.4x4.5 props
#define UAV_DRAG_COEFF 2.0
#define UAV_MASS 	1.28
#define UAV_LENGTH 	0.248
#define UAV_WIDTH 	0.248
#define UAV_HEIGHT 	0.2

// Bus settings
#define I2C_CLK_FREQ 400000
#define PWM_CLK_FREQ 1000
#define UAVCAN_CLK_FREQ 1000000

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
	FrameGenerator* traffic;

	LSM6DS3* sensor_acc_gyro;
	LIS3MDL* sensor_mag;
	LPS22HB* sensor_baro;
	A2235H* sensor_gps;

	Top(sc_module_name name) : sc_module(name)
	{
		// CAN Bus
		can_bus = new uav_can_bus("CAN_BUS", UAVCAN_CLK_FREQ);

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
		sensor_gps = new A2235H("SENSOR_GPS", 1);
		sensor_gps->canif(*can_bus);

		// UAV Physics Model
		uav = new PhysicsObject(UAV_MASS, UAV_DRAG_COEFF, UAV_LENGTH, UAV_WIDTH, UAV_HEIGHT);
		motor_1 = new Motor("MOTOR_1", Vec3d(UAV_LENGTH/2, UAV_LENGTH/2, 0), true);
		motor_2 = new Motor("MOTOR_2", Vec3d(UAV_LENGTH/2, -UAV_LENGTH/2, 0), false);
		motor_3 = new Motor("MOTOR_3", Vec3d(-UAV_LENGTH/2, -UAV_LENGTH/2, 0), true);
		motor_4 = new Motor("MOTOR_4", Vec3d(-UAV_LENGTH/2, UAV_LENGTH/2, 0), false);
		motor_1->pwm_in(*pwm_bus_1);
		motor_2->pwm_in(*pwm_bus_2);
		motor_3->pwm_in(*pwm_bus_3);
		motor_4->pwm_in(*pwm_bus_4);
		aeromodel = new Aerodynamics();
		uav->addComponent(motor_1);
		uav->addComponent(motor_2);
		uav->addComponent(motor_3);
		uav->addComponent(motor_4);
		uav->addComponent(aeromodel);
		uav->addComponent(sensor_baro);
		uav->addComponent(sensor_acc_gyro);
		uav->addComponent(sensor_mag);

		// Physics Sim
		phys = new PhysicsSim("PHYSICS_SIM");
		phys->addObject(uav);

		// Hardware Accelerator
		hardware = new FlightController("FLIGHT_CONTROL", phys, uav);
		hardware->i2c_mst(*iic_bus);
		hardware->pwm_out_1(*pwm_bus_1);
		hardware->pwm_out_2(*pwm_bus_2);
		hardware->pwm_out_3(*pwm_bus_3);
		hardware->pwm_out_4(*pwm_bus_4);
		hardware->canif(*can_bus);

		// Traffic generator (UAVCAN)
		traffic = new FrameGenerator("TRAFFIC_GEN", 20, 15);
		traffic->canif(*can_bus);
	}
};

// System C Main
int sc_main(int argc, char *argv[]) {
	Config conf;
	string cfilename = "config.txt";

	// Initialize rand
	srand(time(0));

	// Instantiate modules
	Top top_inst("TOP");

	// Load configuration file
	if(argc > 1)
		cfilename = argv[1];
	cout << "Loading configuration file from '" << cfilename << "'..." << endl;
	if(!conf.load(cfilename)) {
		cout << "Count not load configuration file" << endl;
		return 0;
	}

	// PID configs
	double pid[6];
	conf.getDouble("pid_att_roll", pid, 6);
	top_inst.hardware->attPID[ROLL].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_att_pitch", pid, 6);
	top_inst.hardware->attPID[PITCH].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_att_yaw", pid, 6);
	top_inst.hardware->attPID[YAW].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_rate_roll", pid, 6);
	top_inst.hardware->ratePID[ROLL].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_rate_pitch", pid, 6);
	top_inst.hardware->ratePID[PITCH].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_rate_yaw", pid, 6);
	top_inst.hardware->ratePID[YAW].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_pos_x", pid, 6);
	top_inst.hardware->posPID[XAXIS].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_pos_y", pid, 6);
	top_inst.hardware->posPID[YAXIS].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_pos_z", pid, 6);
	top_inst.hardware->posPID[ZAXIS].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_vel_x", pid, 6);
	top_inst.hardware->velPID[XAXIS].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_vel_y", pid, 6);
	top_inst.hardware->velPID[YAXIS].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);
	conf.getDouble("pid_vel_z", pid, 6);
	top_inst.hardware->velPID[ZAXIS].initialize(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);

	// Physics options
	top_inst.phys->setTiming(conf.getDouble("physics_sim_delta"), conf.getDouble("physics_sim_time"));
	if(conf.getBool("log_physics_enable"))
		top_inst.uav->enableLog(conf.getString("log_physics_file"));

	// I2C Options
	top_inst.iic_bus->setFrequency(conf.getDouble("i2c_bus_frequency"));
	top_inst.iic_bus->showTraffic(conf.getBool("i2c_bus_verbose"));
	top_inst.hardware->setMagRate(conf.getDouble("mag_read_rate"));
	top_inst.hardware->setAccGyroRate(conf.getDouble("acc_gyro_read_rate"));
	top_inst.hardware->setBaroRate(conf.getDouble("baro_read_rate"));

	// Flight test options
	top_inst.proc->setFlightTest(conf.getInt("flight_test_type"), conf.getDouble("flight_test_interval"));
	top_inst.proc->showCommands(conf.getBool("flight_test_verbose"));
	if(conf.getBool("flight_test_program_enable")) {
		top_inst.proc->enableProgram(conf.getString("flight_test_program_file"));
	}

	// UAV Options
	top_inst.uav->mass = conf.getDouble("uav_mass");
	top_inst.uav->drag_coeff = conf.getDouble("uav_drag_coeff");
	top_inst.uav->position.set(0,0,conf.getDouble("uav_initial_height"));
	top_inst.motor_1->setPropeller(conf.getDouble("uav_prop_diameter"), conf.getDouble("uav_prop_pitch"));
	top_inst.motor_1->setMotorLimits(conf.getDouble("uav_motor_max_rpm"), conf.getDouble("uav_motor_max_torque"));
	top_inst.motor_2->setPropeller(conf.getDouble("uav_prop_diameter"), conf.getDouble("uav_prop_pitch"));
	top_inst.motor_2->setMotorLimits(conf.getDouble("uav_motor_max_rpm"), conf.getDouble("uav_motor_max_torque"));
	top_inst.motor_3->setPropeller(conf.getDouble("uav_prop_diameter"), conf.getDouble("uav_prop_pitch"));
	top_inst.motor_3->setMotorLimits(conf.getDouble("uav_motor_max_rpm"), conf.getDouble("uav_motor_max_torque"));
	top_inst.motor_4->setPropeller(conf.getDouble("uav_prop_diameter"), conf.getDouble("uav_prop_pitch"));
	top_inst.motor_4->setMotorLimits(conf.getDouble("uav_motor_max_rpm"), conf.getDouble("uav_motor_max_torque"));
	if(conf.getBool("log_motor_enable"))
		top_inst.hardware->enableLog(conf.getString("log_motor_file"));

	// UAVCAN options
	top_inst.can_bus->setFrequency(conf.getDouble("uavcan_bus_frequency"));
	top_inst.can_bus->showTraffic(conf.getBool("uavcan_bus_verbose"));
	top_inst.traffic->setRate(conf.getDouble("uavcan_traffic_rate_range"), conf.getDouble("uavcan_traffic_rate_min"));
	top_inst.traffic->setSize(conf.getDouble("uavcan_traffic_size_range"), conf.getDouble("uavcan_traffic_size_min"));
	top_inst.traffic->enable(conf.getBool("uavcan_traffic_enable"));
	top_inst.sensor_gps->setBroadcastRate(conf.getDouble("gps_broadcast_rate"));
	top_inst.sensor_gps->enableBroadcast(conf.getBool("gps_broadcast_enable"));

	// Start simulation
	cout << "Beginning simulation..." << endl << endl;
	sc_start();
	cout << "[" << sc_time_stamp() << "] Simulation stopped." << endl;

	// Close streams
	top_inst.uav->disableLog();
	top_inst.hardware->disableLog();
	top_inst.proc->disableProgram();

	// Print data
	cout << endl;
	cout << "---------------------------------------------" << endl;
	cout << "Statistics" << endl;
	cout << "---------------------------------------------" << endl;
	cout << "UAVCAN bus utilization \t\t= " << top_inst.can_bus->utilization()*100.0 << "%" << endl;
	cout << "UAVCAN bus message count \t= " << top_inst.can_bus->numTransactions() << endl;
	cout << "I2C bus utilization \t\t= " << top_inst.iic_bus->utilization()*100.0 << "%" << endl;
	cout << "I2C bus message count \t\t= " << top_inst.iic_bus->numTransactions() << endl;
	return 0;
}
