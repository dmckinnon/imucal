#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "IMU.h"

// Tunable parameters
#define ALLAN_AVERAGE_INTERVAL_SECONDS 1
#define STATIC_THRESHOLD_MULTIPLIER 6

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
		ComputeAllanVariance(i, numSamplesInInterval, samples, accelAllanVariance, gyroAllanVariance);

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
*	Calibrate the IMU
* 
*/
bool IMU::Calibrate(const int initTime, const int freq)
{
	// sanity checks
	if (samples.size() < freq * initTime)
	{
		return false;
	}

	//std::vector<Vector3f> accelSamples

	// Go over initTime samples and compute the Allan variance
	// This gives a threshold for stationary moments, used below.
	// We also use this initial period to compute the gyro biases.
	// The sample time for averages in this computation is defined
	// as ALLAN_AVERAGE_INTERVAL_SECONDS in CalibrationHelpers.h
	Eigen::Vector3f allanVariance;
	// TODO specify which device? Do for accel, and then also for gyro
	int numIntervals = initTime / ALLAN_AVERAGE_INTERVAL_SECONDS;
	int numSamplesInInterval = freq * ALLAN_AVERAGE_INTERVAL_SECONDS;
	//ComputeAllanVariance(numIntervals, numSamplesInInterval, allanVariance);
	float staticThreshold = STATIC_THRESHOLD_MULTIPLIER * allanVariance.norm() * allanVariance.norm();


	// Why is allan varaince for gyro computed?
	// don't we need this for accel? TO compare to accel?

	// Use Allan variance  + graph this to find a good T_init
	// make a function that does this



	// average over each axis in this time to compute the biases
	int numInitSamples = freq * initTime;
	for (int i = 0; i < numInitSamples; ++i)
	{
		// TODO: watch for numerical growth here, this could get large?
		// ort lose precision
		bias += samples[i].gyro;
	}
	bias /= numInitSamples;



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
	std::vector<IMUSample>& inputSamples,
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
		priorAvg += inputSamples[index].accel;
	}
	priorAvg /= numSamplesInInterval;

	for (int i = 1; i < numIntervals; ++i)
	{
		// Compute current prior average
		currentAvg.setZero();
		for (; index < i*numSamplesInInterval; ++index)
		{
			currentAvg += inputSamples[index].accel;
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
		priorAvg += inputSamples[index].gyro;
	}
	priorAvg /= numSamplesInInterval;

	for (int i = 1; i < numIntervals; ++i)
	{
		// Compute current prior average
		currentAvg.setZero();
		for (; index < i * numSamplesInInterval; ++index)
		{
			currentAvg += inputSamples[index].gyro;
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