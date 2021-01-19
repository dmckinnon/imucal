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
		computeInitVariance = false;
		logIMUData = false;
		calibrate = false;
		outputFile = "";
		inputFile = "";
	}

	bool ParseArgs(int argc, char* argv[]);

	int initTime;
	int captureFrequency;
	bool logIMUData;
	bool calibrate;
	bool computeInitVariance;
	std::string inputFile;
	std::string outputFile;
};