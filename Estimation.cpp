#include "Estimation.h"

using namespace Eigen;
using namespace std;

/***************************************************************
  Runge-Kutta Integration
  This is an iterative algorithm for integrating a time-series signal.
  Yes, we could just take each value and divide by each time step and add them up,
  but this is better, especially when the rate of change is dependent on the
  changing variable. The raw theory is here: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
  Since Tedaldi et al works in quaternions, so will I. 
******/
bool IntegrateWithRK4(
	const vector<Vector3f> signal,
	const float timeStep,
	vector<Vector3f>& output)
{
	vector<Vector4f> orientations;
	float h = timeStep;
	output.clear();
	if (signal.size() < 2)
	{
		return false;
	}

	// Assume rotation at start time is the origin. We can rotate by an offset later
	Vector4f r0;
	r0.setZero();
	orientations.push_back(r0);
	float t0 = 0;

	Vector4f k1, k2, k3, k4;
	float ti = t0;
	float one_sixth = 1.0f / 6.0f;
	for (int i = 1; i < signal.size(); ++i)
	{
		ti += h;
		Vector4f ri = orientations.back();


		Vector4f r1 = ri;
		k1 = GetRotationalVelocity(r1, ti, signal);
		Vector4f r2 = ri + h * 0.5 * k1;
		k2 = GetRotationalVelocity(r2, ti + 0.5 * timeStep, signal);
		Vector4f r3 = ri + h * 0.5 * k2;
		k3 = GetRotationalVelocity(r3, ti + 0.5 * timeStep, signal);
		Vector4f r4 = ri + h * k3;
		k4 = GetRotationalVelocity(r4, ti + timeStep, signal);

		ri += one_sixth * h * (k1 + 2*k2 + 2*k3 + k4);

		orientations.push_back(ri);
	}

	// convert output to SE3 algebra elements?
	// convert from quaternion to euler angles which is the lie algebra
	// then perform exp to go from algebra to group
	// output = orientations
	for (int i = 0; i < orientations.size(); ++i)
	{
		Vector4f q = orientations[i];
		float phi = atan2f(2*(q(0)*q(1) + q(2)*q(3)), q(0)*q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3));
		float theta = -asinf(2*(q(1)*q(3) - q(0)*q(2)));
		float psi = atan2f(2 * (q(0) * q(3) + q(1) * q(2)), q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3));

		// create a unit vector of this to denote direction
		Vector3f direction;
		direction << phi, theta, psi;
		direction /= direction.norm();
		output.push_back(direction);
	}

	return true;
}

/*
 * From Tedaldi et al, Equation 15 
 */
Vector4f GetRotationalVelocity(
	const Vector4f rotation,
	const float time,
	const vector<Vector3f> signal)
{
	Vector4f vel;
	vel.setZero();

	// interpolate the rotational velocity from the signal given the time
	int lowerIndex = floor(time);
	int upperIndex = ceil(time);

	Vector3f w = (upperIndex - time) * signal[lowerIndex] + (time - lowerIndex) * signal[upperIndex];
	w /= 2;

	Matrix4f skew;
	skew << 0,   -w(0), -w(1), -w(2),
		   w(0),   0,    w(2), -w(1),
		   w(1), -w(2),   0,    w(0),
		   w(2),  w(1), -w(0),   0;

	vel = 0.5 * skew * rotation;

    return vel;
}