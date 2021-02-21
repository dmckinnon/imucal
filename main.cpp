#include <iostream>
#include <fstream>
#include "Config.h"
#include "IMU.h"


using namespace std;

/*
	This program is designed to find the calibration parameters - the bias and gain - 
	for an IMU. I've written and debugged it using an MPU6050, but theoretically it
	will work for any IMU. 
	The algorithm and theory I'm working from comes from "A Robust and Easy to Implement Method
	for IMU Calibration without External Equipment" by Tedaldi, Pretto, Menegatti, '13: 
	https://www.researchgate.net/publication/273383944_A_robust_and_easy_to_implement_method_for_IMU_calibration_without_external_equipments

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
	IMU imu;

	Config config;
	if (!config.ParseArgs(argc, argv))
	{
		exit(-1);
	}

	// 'l' is log imu data
	// 'o' specifies the output file for the log
	// 'c' is calibrate 
	// 'i' is input file to be calibrated
	// if no input file is provided, then assume that they are calibrating the data about to be logged
	if (config.logIMUData)
	{
		cout << "Logging data. This will guide you through the data capture procedure. " << endl;
		if (!imu.ConnectAndLogFromArduino(config.comPort))
		{	
			cout << "Could not read data from com port. " << endl;
		}

		if (!config.outputFile.empty())
		{
			// Write the IMU log out to the specified file
			imu.WriteLogToFile(config.outputFile);
		}
	}

	if (config.computeInitVariance)
	{
		// If we don't already have log data, use input file
		if (!imu.HasData())
		{
			if (imu.ReadLogFromFile(config.inputFile))
			{
				std::cout << "Could not get data from input file " << config.inputFile << std::endl;
			}
		}

		cout << "Computing Allan Variances from data and saving to " << config.varianceFile << endl;
		imu.ComputeAndWriteDataForInitialisationPeriod(config.varianceFile);
	}

	if (config.calibrate)
	{
		cout << "Calibrating IMU data. If data was just logged, that will be calibrated." << endl
			<< "Otherwise, the provided input file will be calibrated. If no source exists, program will exit." << endl;

		std::ifstream inputFile(config.inputFile);
		if (imu.HasData())
		{
			cout << "Using existing log data." << endl;
		}
		else if (inputFile.is_open())
		{
			cout << "Attempting to read from file." << endl;
			if (imu.ReadLogFromFile(config.inputFile))
			{
				std::cout << "Could not get data from input file " << config.inputFile << std::endl;
			}
		}
		else
		{
			cout << "No source provided. Exiting." << endl;
			return -1;
		}

		if (!imu.Calibrate(
			        config.initTime,
					config.captureFrequency,
					config.staticTime,
					config.transferTime))
		{
			std::cout << "Failed to calibrate" << std::endl;
			exit(-1);
		}

		// Write calibration to file
		imu.WriteCalibrationToFile("imucal.txt");
	}


	return 0;
}