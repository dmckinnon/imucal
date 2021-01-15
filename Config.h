#pragma once
#include <string>

/*
	Class for configuration parameters, and any IO ops, that IMU Calibration might need.
*/
class Config
{
public:
	Config()
	{
		logIMUData = false;
		calibrate = false;
		outputFile = "";
		inputFile = "";
	}

	bool ParseArgs(int argc, char* argv[]);

	bool logIMUData;
	bool calibrate;
	std::string inputFile;
	std::string outputFile;
};