#include <iostream>

using namespace std;

/*
	This program is designed to find the calibration parameters - the bias and gain - 
	for an IMU. I've written and debugged it using an MPU6050, but theoretically it
	will work for any IMU. 

	Note that for now all of the following is theoretical as I haven't yet implemented everything.
	I find it helpful to write this as a series of goals, so this is what I intend to achieve. 

	I use an arduino to read from this IMU and stream to serial, read here using 
	https://github.com/manashmandal/SerialPort this serial port C++ library, all
	credit for that to manashmandal (used under license ... githuib something something)
	This is the 'log' mode mentioned below. This program attempts to detect when the 
	IMU is still and guide the user through the calibration procedure. Since holding
	an IMU steady in at least 9 different positions is difficult, I 3D printed a polyhedron
	and stuck the IMU to one side of it, and sat this on each of its flat surfaces.

	The other mode is 'calibrate', which takes a (hopefully valid) log file. There is a
	validation step, which is essentially the same as the guidance procedure above. Then
	the program uses these data to calibrate the gain and offset for the IMU as well as
	the transform from gyro to accel. 

	This is essentially just an optimisation problem, and everything else in the procedure is
	either getting enough distinct data, or getting the data into the right form, once the theory
	is sorted out. 


*/
int main(int argc, char* argv[])
{
	// Preamble, and user options:
	cout << "user options" << endl;

	// await user input


	// 'l' is log imu data
	// 'o' specifies the output file for the log
	// 'c' is calibrate 
	// 'i' is input file to be calibrated
	// if no input file is provided, then assume that they are calibrating the data about to be logged


	return 0;
}