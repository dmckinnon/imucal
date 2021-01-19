#include <iostream>
#include <stdlib.h>
#include "Config.h"

using namespace std;

bool Config::ParseArgs(int argc, char* argv[])
{
	for (int i = 1; i < argc; ++i)
	{
		if (argv[i] == "-l")
		{
			logIMUData = true;
		}
		else if (argv[i] == "-o")
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
		else if (argv[i] == "-i")
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
		else if (argv[i] == "-c")
		{
			calibrate = true;
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
		else if (argv[i] == "-f")
		{
			if (i + 1 < argc)
			{
				captureFrequency = strtof(argv[++i], nullptr);
			}
			else
			{
				std::cout << "No frequency specified!" << std::endl;
			}
		}
		else
		{
			std::cout << "Invalid input!" << std::endl;
			std::cout << "Correct usage:\n";
			std::cout << "";
		}
	}

	return true;
}