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
		calibrate = true;
		staticTime = 4;
		transferTime = 1;
		outputFile = "";
		inputFile = "";
		varianceFile = "";
		comPort = "COM1";
	}

	bool ParseArgs(int argc, char* argv[]);

	int initTime;
	int captureFrequency;
	bool logIMUData;
	bool calibrate;
	bool computeInitVariance;
	float staticTime;
	float transferTime;
	std::string inputFile;
	std::string outputFile;
	std::string varianceFile;
	std::string comPort;
};