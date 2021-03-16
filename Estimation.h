#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

/*
 * This file defines methods for estimating signals and variables.
 * The main methods are Runge-Kutta integration and Levenberg-Marquardt estimation,
 * and most of the other methods exist to support these
 */

/*
 * Levenberg-Marquardt specifically for accelerometer parameter estimation
 * This means it has the jacobian hardcoded and the parameters hardcoded
 */
bool EstimateAccelerometerParams(
	const std::vector<Eigen::Vector3f> accelVals,
	const float g,
	Eigen::Vector3f& pose,
	Eigen::Vector3f& scale,
	Eigen::Vector3f& bias);

/*
 * Runge Kutta Integration
 * and helper methods
 * The algorithm comes from https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
 */
bool IntegrateWithRK4(
	const std::vector<Eigen::Vector3f> signal,
	const float timeStep,
	std::vector<Eigen::Vector3f>& output);

Eigen::Vector4f GetRotationalVelocity(Eigen::Vector4f rotation, float time, const std::vector<Eigen::Vector3f> signal);