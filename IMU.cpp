#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "IMU.h"

IMU::IMU() :
	rot_yz(0),
	rot_zy(0),
	rot_zx(0)
{
	std::vector<double> scaleInit(3, 1.0);
	std::vector<double> biasInit(3, 0.0);

	scale.setZero();
	bias.setZero();
}

bool IMU::HasData()
{
	if (samples.size() != 0)
	{
		return true;
	}

	// Maybe there are other criteria here

	return false;
}

bool IMU::Calibrate(const int initTime, const float freq)
{
	// Go over initTime samples and compute the Allan variance
	// This gives a threshold for stationary moments, used below.
	// We also use this initial period to compute the gyro biases.


	// Find the timestamps of all the stationary points.
	// Do we auto-detect how long they are? I like that ...

	// Sliding window of 1 second of samples to compute variance across that window. 
	// When variance is below the 'stationary' threshold, set time at start of window to be 'start of stationary'
	// and as soon things are above threshold, the time at the end of the prior window is the end of stationary

	return true;
}

bool IMU::WriteCalibrationToFile(std::string filename)
{
	std::ofstream calFile(filename.c_str());
	if (calFile.is_open())
	{
		// Cal file format is:
		calFile << "rot_yz: " << rot_yz << std::endl;
		calFile << "rot_yz: " << rot_yz << std::endl;
		calFile << "rot_yz: " << rot_yz << std::endl;
		calFile << "scale: " << scale[0] << "," << scale[1] << "," << scale[2] << std::endl;
		calFile << "bias: " << bias[0] << "," << bias[1] << "," << bias[2] << std::endl;

		return true;
	}

	return false;
}

void IMU::WriteLogToFile(std::string filename)
{
	std::ofstream logFile(filename.c_str());
	if (logFile.is_open())
	{
		// Write csv header
		logFile << "timestamp,accel0,accel1,accel2,gyro0,gyro1,gyro2" << std::endl;

		for (int i = 0; i < samples.size(); ++i)
		{
			logFile << samples[i].timestamp << ","
				    << samples[i].accel(0) << "," << samples[i].accel(1) << "," << samples[i].accel(2) << "," 
				    << samples[i].gyro(0) << "," << samples[i].gyro(1) << "," << samples[i].gyro(2) << std::endl;
		}
	}
}

void IMU::ReadLogFromFile(std::string filename)
{
	std::ifstream logFile(filename.c_str());
	if (logFile.is_open())
	{
		std::string line;
		bool firstLine = true;
		while (std::getline(logFile, line))
		{
			if (firstLine)
			{
				firstLine = false;
				continue;
			}

			if (line.size() < 1)
			{
				continue;
			}
			
			char* values;
			IMUSample sample;
			sample.timestamp = strtoull(line.c_str(), &values, 10);
			sample.accel(0) = strtof(values, &values);
			sample.accel(1) = strtof(values, &values);
			sample.accel(2) = strtof(values, &values);
			sample.gyro(0) = strtof(values, &values);
			sample.gyro(1) = strtof(values, &values);
			sample.gyro(2) = strtof(values, &values);

			// Debug point to confirm this all happened correctly
		}
	}
}