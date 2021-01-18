#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

/*
	Class to represent an IMU for IMU calibration.
	Contains data from a log, and can write this out or read it in from a file.
	Contains the parameters to be estimated, and can write these to a file.

	// TODO: have a verification method for existing parameters to compare?

	Contains the Calibration functions to estimate these parameters
*/
class IMU
{
public:
	IMU();
	~IMU();

	// Logging?
	// use a class that has IO?
	bool HasData();

	bool Calibrate();
	bool WriteCalibrationToFile(std::string filename);

	void WriteLogToFile(std::string filename);
	void ReadLogFromFile(std::string filename);


private:

	struct IMUSample
	{
		Eigen::Vector3f accel;
		Eigen::Vector3f gyro;
		
		uint64_t timestamp;
	};

	std::vector<IMUSample> samples;

	// parameters to estimate
	double rot_yz;
	double rot_zy;
	double rot_zx;
	Eigen::Vector3f scale;
	Eigen::Vector3f bias;
};