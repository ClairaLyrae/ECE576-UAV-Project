------------------------------------------------------
ECE576 Final Project - Hardware Accelerated UAV System 
------------------------------------------------------

Compilation
------------------------------------------------------
The project uses a vector math library called GMTL, which is included
alongside the source for compilation. Additionally, MATLAB source for
plotting outputs from the simulation are provided in the /matlab folder.

A folder containing a makefile suitable for use on the ece server is provided
in the /remote directory. This makefile should be placed in the source folder.
The GMTL library must be placed in the parent directory of the source folder. 

Execution:
------------------------------------------------------
All options in the simulation can be adjusted by use of a configuration file. The
default configuration file is config.txt, but another file can be specified as the 
first argument when running the program. See the configuration file for details on
what kind of options can be set.

Inside the configuration file, a optional flight program file can be specified. This
file contains a list of commmands for the processor to execute, and can be used
to generate arbitrary flight sequences for testing.


Output:
------------------------------------------------------
Output is provided in the form of two optional log files. These are tab-delimited 
printouts of the sim time, UAV position, velocity, attitude, attitude rate, up vector,
and forward vector. All vectors are given in three components. Attitude is given in
the ROLL, PITCH, YAW form. Subsesquent time steps are given on each line. 

A motor control log is provided in the same fashion.