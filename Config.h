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
		initTime = 60;
		captureFrequency = 100;
		logIMUData = false;
		calibrate = false;
		outputFile = "";
		inputFile = "";
	}

	bool ParseArgs(int argc, char* argv[]);

	int initTime;
	float captureFrequency;
	bool logIMUData;
	bool calibrate;
	std::string inputFile;
	std::string outputFile;
};