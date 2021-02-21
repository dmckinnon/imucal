#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "IMU.h"
#include "..\SerialPort\include\SerialPort.hpp"

using namespace std::chrono;

// Tunable parameters
#define ALLAN_AVERAGE_INTERVAL_SECONDS 1
#define STATIC_THRESHOLD_MULTIPLIER 6

// Non-tunable params
#define MAX_SERIAL_DATA_LENGTH 255
#define ARDUINO_PACKET_LENGTH 24

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

IMU::~IMU()
{

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

/*
*	Compute the Allan Variance for a data stream and write this out to a file.
*   The definition for Allan variance can be found here: https://en.wikipedia.org/wiki/Allan_variance#Allan_variance
*   This is used to see when the noise in the data settles from bias drifts, to determine
*   a good length of time for initialisation.
*   The idea is that for every time interval of length ALLAN_AVERAGE_INTERVAL_SECONDS we compute the average
*   and then we take half the average of squared differences between consecutive averages. 
*   Thus, if there is a large difference between these interval averages, the signal is obviously drifting;
*   when the difference becomes small, the signal has settled.
*/
void IMU::ComputeAndWriteDataForInitialisationPeriod(std::string filename)
{
	if (samples.size() == 0)
	{
		std::cout << "No data to write to file!" << std::endl;
		return;
	}

	// Increase interval size, and for each interval size compute the allan variance and log it
	std::vector<Eigen::Vector3f> accelAllanVariances;
	std::vector<Eigen::Vector3f> gyroAllanVariances;

	// The init time is the number of samples in the entire sequence
	int initTime = (samples[samples.size() - 1].timestamp - samples[0].timestamp)/1000;
	float freq = 1 / (float)(samples[1].timestamp - samples[0].timestamp);
	int numIntervals = initTime / ALLAN_AVERAGE_INTERVAL_SECONDS;
	int numSamplesInInterval = freq * ALLAN_AVERAGE_INTERVAL_SECONDS;

	for (int i = 1; i <= numIntervals; ++i)
	{
		// Compute the Allan Variance for both accel and gyro
		Eigen::Vector3f accelAllanVariance, gyroAllanVariance;
		ComputeAllanVariance(i, numSamplesInInterval, accelAllanVariance, gyroAllanVariance);

		accelAllanVariances.push_back(accelAllanVariance);
		gyroAllanVariances.push_back(gyroAllanVariance);
	}


	// Write these out to a file for analysis
	std::ofstream output(filename);
	if (output.is_open())
	{
		output << "numIntervals,accelVar0,accelVar1,accelVar2,gyroVar0,gyroVar1,gyroVar2" << std::endl;
		for (int i = 0; i < numIntervals; ++i)
		{
			Eigen::Vector3f aav = accelAllanVariances[i];
			Eigen::Vector3f gav = gyroAllanVariances[i];
			output << i << "," << aav[0] << "," << aav[1] << "," << aav[2] << "," << gav[0] << "," << gav[1] << "," << gav[2] << std::endl;
		}
		std::cout << "Written Allan variances for accel and gyro over a time period of " << initTime << " seconds to " << filename << std::endl;
	}
	else
	{
		std::cout << "Cannot open output file " << filename << std::endl;
	}
}

/*
 * Connect to the Arduino via COM ports, and read the IMU data from the byte stream.
 * 
 * Note that since I am writing this on windows, there is some hardcoding for windows COM ports.
 * All SerialPort code comes from https://github.com/manashmandal/SerialPort and all credit to them.
 */
bool IMU::ConnectAndLogFromArduino(std::string comPort)
{
	std::string portName = "\\\\.\\" + comPort;
	SerialPort* arduino= new SerialPort(portName.c_str());

	while (true) {
		std::cout << "Searching in progress";
		// wait connection
		while (!arduino->isConnected())
		{
			Sleep(100);
			std::cout << ".";
			arduino = new SerialPort(portName.c_str());
		}

		// Checking if arduino is connected or not
		if (arduino->isConnected())
		{
			std::cout << std::endl << "Connection established at port " << portName << std::endl;
			std::cout << "Press 'C' to stop reading." << std::endl;
		}
		else
		{
			std::cout << "Cannot connect to port " << portName << std::endl;
			break;
		}

		// Read in data until arduino is disconnected or a key is pressed
		while (arduino->isConnected())
		{
			char bytestream[MAX_SERIAL_DATA_LENGTH];
			if (arduino->readSerialPort(bytestream, ARDUINO_PACKET_LENGTH))
			{
				// Read the data from the bytestream. There are 24 bytes, in the format
				// of accel xyz, gyro xyz, so six floats. We convert the string to a float array
				float* input = (float*)bytestream;
				IMUSample sample;
				sample.accel << input[0], input[1], input[2];
				sample.gyro << input[3], input[4], input[5];
				milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
				sample.timestamp = ms.count();

				samples.push_back(sample);
			}
			else
			{
				std::cerr << "Error occurred reading data" << std::endl;
			}

			// check for keypress and close port
			// This is windows-hardcoded, sorry
			if (GetKeyState('C') & 0x8000/*Check if high-order bit is set (1 << 15)*/)
			{
				std::cout << "Closing serial port" << std::endl;
				arduino->closeSerial();
				return true;
			}
			
		}
	}

	return false;
}

/*
 *	Calibrate the IMU
 * 
 *   Theory goes here
 */
bool IMU::Calibrate(
	const int initTime,
	const int freq,
	const float staticIntervalTime,
	const float transitionTime)
{
	// sanity checks
	if (samples.size() < freq * initTime)
	{
		return false;
	}


	// ------------------------------------------------------------------------
	// Compute thresholds for static detector
	// Go over initTime samples and compute the Allan variance
	// This gives a threshold for stationary moments, used below.
	// We also use this initial period to compute the gyro biases.
	// The sample time for averages in this computation is defined
	// as ALLAN_AVERAGE_INTERVAL_SECONDS above
	Eigen::Vector3f allanVarianceAccel, allanVarianceGyro;
	int numIntervals = initTime / ALLAN_AVERAGE_INTERVAL_SECONDS;
	int numSamplesInInterval = freq * ALLAN_AVERAGE_INTERVAL_SECONDS;
	ComputeAllanVariance(numIntervals, numSamplesInInterval, allanVarianceAccel, allanVarianceGyro);
	float staticThresholdAccel = STATIC_THRESHOLD_MULTIPLIER * allanVarianceAccel.norm() * allanVarianceAccel.norm();
	std::cout << "Static motion threshold for acceleration data is " << staticThresholdAccel << std::endl;

	// ------------------------------------------------------------------------
	// Detect static sections and moving sections
	// These will be stored as pairs of indices into the data array that are the first
	// and last indices of the valid static section. Anything between two valid static sections
	// is a valid moving section.
	// A static section is considered valid if, for the given static interval from the config args,
	// the variance of the accel data is below the staticThresholdAccel computed above.
	// At least 9 static intervals are required for a minimal calibration
	std::vector<std::pair<int, int>> staticIntervals;
	ComputeStaticIntervals(staticThresholdAccel, staticIntervalTime, transitionTime, initTime, staticIntervals);
	std::cout << "Found " << staticIntervals.size() << " static intervals of length " << staticIntervalTime << " in the data" << std::endl;
	if (staticIntervals.size() < 9)
	{
		std::cout << "Fewer than 9 static intervals found! Not enough to properly calibrate. Aborting." << std::endl;
		return false;
	}

	// ------------------------------------------------------------------------
	// Compute gyro biases
	// average over each axis in this time to compute the biases
	int numInitSamples = freq * initTime;
	bias = samples[0].gyro;
	for (int i = 1; i <= numInitSamples; ++i)
	{
		bias *= i;

		bias += samples[i].gyro;

		// To avoid numerical growth and losing precision,
		// we employ a rolling average. Multiply by prior denominator,
		// and divide by current sample size to maintain an accurate average. 
		// eg. (x+y)/2 is the avg of 2. To get avg of 3, (x+y)/2 * 2 + z all divided by 3, gives (x+y+z)/3
		bias /= (float)(i + 1);
	}


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

bool IMU::ReadLogFromFile(std::string filename)
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
	else
	{
		return false;
	}

	return HasData();
}

/*
*	Compute the Allan variance for a sequence of IMU samples,
*	given the samples and the interval for averaging.
*   https://en.wikipedia.org/wiki/Allan_variance#Allan_variance
*   https://ui.adsabs.harvard.edu/abs/2006MeScT..17.2980S/abstract
* 
*   Yes, this could be written more streamlined for performance, but
*   my focus here is on readability and understanding, hence breaking
*   up accel and gyro
*/
void IMU::ComputeAllanVariance(
	const int numIntervals,
	const int numSamplesInInterval,
	Eigen::Vector3f& allanVarianceAccel,
	Eigen::Vector3f& allanVarianceGyro)
{
	allanVarianceAccel.setZero();
	allanVarianceGyro.setZero();

	//--------------------------------------------------------

	// Compute accel Allan Variance
	Eigen::Vector3f priorAvg, currentAvg;
	currentAvg.setZero();
	priorAvg.setZero();
	int index = 0;

	// compute first prior average
	for (; index < numSamplesInInterval; ++index)
	{
		priorAvg += samples[index].accel;
	}
	priorAvg /= numSamplesInInterval;

	for (int i = 1; i < numIntervals; ++i)
	{
		// Compute current prior average
		currentAvg.setZero();
		for (; index < i*numSamplesInInterval; ++index)
		{
			currentAvg += samples[index].accel;
		}
		currentAvg /= numSamplesInInterval;

		Eigen::Vector3f diff = currentAvg - priorAvg;
		Eigen::Vector3f sq;
		sq << diff(0) * diff(0), diff(1)* diff(1), diff(2)* diff(2);

		allanVarianceAccel += sq;

		priorAvg = currentAvg;
	}

	allanVarianceAccel /= 2 * numIntervals;

	//--------------------------------------------------------

	// compute gyro Allan Variance
	currentAvg.setZero();
	priorAvg.setZero();
	index = 0;

	// compute first prior average
	for (; index < numSamplesInInterval; ++index)
	{
		priorAvg += samples[index].gyro;
	}
	priorAvg /= numSamplesInInterval;

	for (int i = 1; i < numIntervals; ++i)
	{
		// Compute current prior average
		currentAvg.setZero();
		for (; index < i * numSamplesInInterval; ++index)
		{
			currentAvg += samples[index].gyro;
		}
		currentAvg /= numSamplesInInterval;

		Eigen::Vector3f diff = currentAvg - priorAvg;
		Eigen::Vector3f sq;
		sq << diff(0) * diff(0), diff(1)* diff(1), diff(2)* diff(2);

		allanVarianceAccel += sq;

		priorAvg = currentAvg;
	}

	allanVarianceGyro /= 2 * numIntervals;
}

/*
 * Compute intervals of static motion amongst the given data
 * These intervals are assumed to be at least as long as the given time in the config,
 * and cannot overlap.
 * An interval is considered valid if the magnitude of the variance for the period is below the given threshold.
 * Two consective intervals must have at least half the transition time provided in the config between them.
 * Intervals are denoted by start and end indices for the data array
 */
void IMU::ComputeStaticIntervals(
	const float threshold,
	const float staticTime,
	const float transitionTime,
	const float initTime,
	std::vector<std::pair<int, int>>& staticIntervals)
{
	// For each section of certain length, find the variance
	// if below, flag it, and move straight to the end fo the section
	// otherwise, start a new section with the next sample
	// otherwise, start a new section with the next sample

	float freq = 1 / (float)(samples[1].timestamp - samples[0].timestamp);
	int numSamplesPerInterval = staticTime * freq;
	int numSamplesPerTransition = transitionTime * freq;
	int startingIndex = initTime * freq;

	// Loop over all possible intervals, starting at the end of the expected initialisation time
	for (int i = startingIndex; i < samples.size() - numSamplesPerInterval; ++i)
	{
		// Strictly enforce interval length here. We could do something more dynamic where it expands an interval
		// until the threshold fails, and then take everything before that, and enforce a minimum size.
		// This is just a simple first pass
		if (IsIntervalStatic(i, i + numSamplesPerInterval, threshold))
		{
			// Once an interval has been found, save it and move the counter
			// at least numSamplesPerTransition indices after the end
			// This enforces that no two static intervals can overlap
			staticIntervals.push_back(std::pair<int, int>(i, i+numSamplesPerInterval));
			i += numSamplesPerInterval + numSamplesPerTransition - 1; // account for for loop increment
		}
	}

	std::cout << "Found " << staticIntervals.size() << " static intervals in the data." << std::endl;
}

/*
 * Helper for computing static intervals. On a given interval, compute
 * the magnitude of the variance across all three dimensions for acceleration.
 * If variance magnitude is less than the threshold, not a static interval
 */
bool IMU::IsIntervalStatic(const int start, const int end, const float threshold)
{
	// Compute variance for acceleration over each dimension
	Eigen::Vector3f accelVar, accelAvg;
	accelAvg.setZero();

	for (int i = start; i <= end; ++i)
	{
		accelAvg += samples[i].accel;
	}
	accelAvg /= start - end;

	for (int i = start; i <= end; ++i)
	{
		auto vec = samples[i].accel - accelAvg;
		accelVar += vec.cwiseProduct(vec);
	}
	accelVar /= start - end;

	float magnitude = accelVar.norm();
	return magnitude < threshold;
}