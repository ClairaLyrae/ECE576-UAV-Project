------------------------------------------------------
ECE576 Final Project - Hardware Accelerated UAV System 
------------------------------------------------------

The project uses a vector math library called GMTL, which is included
alongside the source for compilation. Additionally, MATLAB source for
plotting outputs from the simulation are provided in the /matlab folder.

Running the simulation will give you options to set the physics time 
limits and step, as well as the type of PID test to do. PID tests
are composed of a series of impulsive commands sent to the flight controller
hardware form the processor over the UAVCAN bus. These commands will set a
target for the hardware PID controllers to hit.

Output is provided in the form of two log files. These are tab-delimited 
printouts of the sim time, UAV position, velocity, attitude, attitude rate, up vector,
and forward vector. All vectors are given in three components. Attitude is given in
the ROLL, PITCH, YAW form. Subsesquent time steps are given on each line. 

A motor control log is provided in the same fashion.