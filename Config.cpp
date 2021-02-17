#include <iostream>
#include <stdlib.h>
#include "Config.h"

bool Config::ParseArgs(int argc, char* argv[])
{
	for (int i = 1; i < argc; ++i)
	{
		if (argv[i] == "-log")
		{
			logIMUData = true;
		}
		else if (argv[i] == "-logOut")
		{
			if (i + 1 < argc)
			{
				outputFile = argv[++i];
			}
			else
			{
				std::cout << "No output file specified!" << std::endl;
			}
		}
		else if (argv[i] == "-input")
		{
			if (i + 1 < argc)
			{
				inputFile = argv[++i];
			}
			else
			{
				std::cout << "No input file specified!" << std::endl;
			}
		}
		else if (argv[i] == "-varfile")
		{
			if (i + 1 < argc)
			{
				varianceFile = argv[++i];
				computeInitVariance = true;
			}
			else
			{
				std::cout << "No variance file specified!" << std::endl;
			}
		}
		else if (argv[i] == "-nocal")
		{
			calibrate = false;
		}
		else if (argv[i] == "-init")
		{
			if (i + 1 < argc)
			{
				initTime = strtol(argv[++i], nullptr, 10);
			}
			else
			{
				std::cout << "No init time specified!" << std::endl;
			}
		}
		else if (argv[i] == "-freq")
		{
			if (i + 1 < argc)
			{
				captureFrequency = strtol(argv[++i], nullptr, 10);
			}
			else
			{
				std::cout << "No frequency specified!" << std::endl;
			}
		}
		else if (argv[i] == "-stat")
		{
			if (i + 1 < argc)
			{
				staticTime = strtof(argv[++i], nullptr);
			}
			else
			{
				std::cout << "No static time specified!" << std::endl;
			}
		}
		else if (argv[i] == "-trans")
		{
			if (i + 1 < argc)
			{
				staticTime = strtof(argv[++i], nullptr);
			}
			else
			{
				std::cout << "No static time specified!" << std::endl;
			}
		}
		else
		{
			std::cout << "Invalid input!" << std::endl;
			std::cout << "Correct usage:\n";
			std::cout << "IMUCal.exe [optional args]" << std::endl;
			std::cout << "Args are:   " << std::endl
				      << "-log                  : if specified, will log imu data from connected IMU. Off by default" << std::endl
					  << "-logOut               : file to save logged IMU data to. Saves in csv format" << std::endl
					  << "-input                : if specified, will use IMU data from this input file. Logged data has priority over this" << std::endl
				      << "-nocal                : if specified, calibration won't happen. By default program calibrates" << std::endl
					  << "-init                 : Time in seconds to take to initialise calibration period. Default 60 seconds" << std::endl
				      << "-freq                 : Frequency of IMU samples. Default 100Hz" << std::endl
				      << "-varfile              : File to save the Allan variance values from a static Allan Variance test to. Uses given log or input file data" << std::endl
				      << "                        Providing this means that Allan variance calculation will occur. Does not occur by default." << std::endl 
				      << "-stat                 : Time to assume static positions for guided calibration. Default 4 seconds" << std::endl
				      << "-trans                : Time allowed for transitions between static periods for guided calibration. Default 1 second" << std::endl
				      << "" << std::endl;
			return false;
		}
	}

	return true;
}