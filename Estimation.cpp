#include "Estimation.h"
#include <iostream>

#define MAX_LM_ITERATIONS 10
#define REFINEMENT_THRESHOLD (1e-3)

using namespace Eigen;
using namespace std;

/***************************************************************
  Levenberg-Marquardt Estimation for accelerometers
  See http://ethaneade.com/optimization.pdf

  In Tedaldi et al, Eq 10 is what they are trying to minimise.
  Problem is, 
******/
bool EstimateAccelerometerParams(
	const std::vector<Eigen::Vector3f> accelVals,
	const float g,
	Eigen::Vector3f& pose,
	Eigen::Vector3f& scale,
	Eigen::Vector3f& bias)
{
	bool succeeded = true;
	Matrix3f S, T; // scale matrix, pose matrix
	S.setZero();
	S(0, 0) = scale(0);
	S(1, 1) = scale(1);
	S(2, 2) = scale(2);

	T.setZero();
	T << 1, -1*pose(0), pose(1), 
		 0,      1,   -1*pose(2),
		 0,      0,       1; 

	float lambda = 0.1f;
	float prevError = 1000000000; // start error off large so first error is always acceptable
	for (int iterations = 0; iterations < MAX_LM_ITERATIONS; ++iterations)
	{
		// Note that we could use a robust cost function here like Huber cost or Tukey cost to reduce
		// the effect of outliers. I don't because I don't think we will have sufficiently damaging outliers here

		VectorXf update(9);
		float currentError = 0, e = 0;
		MatrixXf JtJ(1, 1); // J.transpose() * J, where J is Jacobian
		VectorXf Jte(9); // J * e, where e is the accumulated error vector

		// accumulate error and jacobians over all measurements
		for (int i = 0; i < accelVals.size(); ++i)
		{
			Vector3f sa_minus_b = S * (accelVals[i] + bias);
			Vector3f transformedAccel = T * sa_minus_b;
			float difference = g*g - transformedAccel.norm() * transformedAccel.norm();
			e = difference * difference;
			currentError += e;

			// Construct the jacobian. The structure of the Jacobian is
			//
			//  J = 2e * -2tA.transpose() * d(tA)/d(update)  
			//
			// where e = e, L in Eq 10
			//       tA = transformedAccel, h in Eq 10
			//       
			// let tA.transpose() * d(tA)/d(update) = J_u, then
			//       
			//        ( -f_y * h(0)                                                                 )
			//        (  f_z * h(1)                                                                 )
			//  J_u = ( -f_z * h(3)                                                                 )
			//        (  (a_x - b_x) * h(0)                                                         )
			//        ( -r(0) * (a_y - b_y) * h(0) + (a_y - b_y) * h(1)                             )
			//        (  r(1) * (a_z - b_z) * h(0) - r(2) * (a_z - b_z) * h(1) + (a_z - b_z) * h(2) )
			//        ( -s_x * h(0)                                                                 )
			//        (  r(0) * s_y * h(0) - s_y * h(1)                                             )
			//        ( -r(1) * s_z * h(0) + r(2) * s_z * h(1) - s_z * h(2)                         )
			//
			// where r = pose vector, s = scale vector, h = tA as above, a = accel vector, b = bias vector
			//       and f = sa - b 
			//
			// So the final value is J = -4e*J_u.transpose()
			//
			auto h = transformedAccel;
			auto f = sa_minus_b;
			auto r = Vector3f(T(0, 1), T(0, 2), T(1, 2));
			MatrixXf J(9, 1);
			J.setZero();
			J(0, 0) = -f(1) * h(0);
			J(1, 0) = f(2) * h(1);
			J(2, 0) = -f(2) * h(3);
			J(3, 0) = (accelVals[i](0) - bias(0)) * h(0);
			J(4, 0) = -r(0) * (accelVals[i](1) - bias(1)) * h(0) + (accelVals[i](1) - bias(1)) * h(1);
			J(5, 0) = r(1) * (accelVals[i](2) - bias(2)) * h(0) - r(2) * (accelVals[i](2) - bias(2)) * h(1) + (accelVals[i](2) - bias(2)) * h(2);
			J(6, 0) = -S(0,0) * h(0);
			J(7, 0) = r(0) * S(1,1) * h(0) - S(1,1) * h(1);
			J(8, 0) = -r(1) * S(2,2) * h(0) + r(2) * S(2,2) * h(1) - S(2,2) * h(2);

			// scale jacobian
			J *= -4 * e;
			
			JtJ += J.transpose() * J;
			Jte += e * J; // Yes, this should be J.tranpose(), but this keeps everything a vertical vector
		}

		// L-M step
		// TODO how to add in covariance?
		for (int i = 0; i < JtJ.rows(); ++i)
		{
			JtJ(i, i) += lambda * JtJ(i, i);
		}

		// compute the total udpate
		update = JtJ.inverse() * Jte;

		// early exit condition
		if (currentError < REFINEMENT_THRESHOLD)
		{
			break;
		}
		else
		{
			// if we get to the final iteration and still error is too high,
			// signal that we did not manage to refine this sufficiently
			// But still return the values
			if (iterations == MAX_LM_ITERATIONS - 1)
			{
				succeeded = false;
			}
		}

		// update and continue
		if (currentError < prevError)
		{
			lambda /= 10;
			prevError = currentError;
		}
		else
		{
			lambda *= 10;
		}

		// Not sure how to perturb the pose properly, need to look this up
		// might need to rotate it
		T(0, 1) += update(0);
		T(0, 2) += update(1);
		T(1, 2) += update(2);
		S(0, 0) += update(3);
		S(1, 1) += update(4);
		S(2, 2) += update(5);
		bias += Vector3f(update(6), update(7), update(8));
	}

	// copy from scale matrix to scale vector
	pose = Vector3f(T(0, 1), T(0, 2), T(1, 2));
	scale = S.diagonal();
	// bias stays the same

	return succeeded;
}

/*
	The purpose of this is to compute the difference between:
	(H + delta_h)(x) - H(x)
	and
	J_H(x)

	This is to test whether or not we have the right formulation of the Jacobian.
	This test verifies that we do.
*/
void FiniteDiff(const Matrix3f& H)
{
	const Vector3f x(1.f, 2.f, 1.f);

	// Let f(h) = Hx
	// Compute f(h+epsilon) and f(h), then divide the difference by epsilon
	Vector3f Hx = H * x;
	float w = Hx(2);
	Hx /= w;
	float e = 0.01f;
	MatrixXf difference(2, 9);
	difference.setZero();
	difference(0, 0) = (((H(0, 0) + e) * x(0) + H(0, 1) * x(1) + H(0, 2) * x(2)) / w - Hx(0)) / e;
	difference(0, 1) = ((H(0, 0) * x(0) + (H(0, 1) + e) * x(1) + H(0, 2) * x(2)) / w - Hx(0)) / e;
	difference(0, 2) = ((H(0, 0) * x(0) + H(0, 1) * x(1) + (H(0, 2) + e) * x(2)) / w - Hx(0)) / e;

	difference(1, 3) = (((H(1, 0) + e) * x(0) + H(1, 1) * x(1) + H(1, 2) * x(2)) / w - Hx(1)) / e;
	difference(1, 4) = ((H(1, 0) * x(0) + (H(1, 1) + e) * x(1) + H(1, 2) * x(2)) / w - Hx(1)) / e;
	difference(1, 5) = ((H(1, 0) * x(0) + H(1, 1) * x(1) + (H(1, 2) + e) * x(2)) / w - Hx(1)) / e;

	float w_e7 = ((H(2, 0) + e) * x(0) + H(2, 1) * x(1) + H(2, 2) * x(2));
	float w_e8 = (H(2, 0) * x(0) + (H(2, 1) + e) * x(1) + H(2, 2) * x(2));
	float w_e9 = (H(2, 0) * x(0) + H(2, 1) * x(1) + (H(2, 2) + e) * x(2));

	float x1 = H(0, 0) * x(0) + H(0, 1) * x(1) + H(0, 2) * x(2);
	float x2 = H(1, 0) * x(0) + H(1, 1) * x(1) + H(1, 2) * x(2);
	difference(0, 6) = (x1 / w_e7 - Hx(0)) / e;
	difference(0, 7) = (x1 / w_e8 - Hx(0)) / e;
	difference(0, 8) = (x1 / w_e9 - Hx(0)) / e;
	difference(1, 6) = (x2 / w_e7 - Hx(1)) / e;
	difference(1, 7) = (x2 / w_e8 - Hx(1)) / e;
	difference(1, 8) = (x2 / w_e9 - Hx(1)) / e;

	// Next, compute the Jacobian using Hartley and Zisserman's method,
	// at x. 
	MatrixXf J(2, 9);
	J.setZero();
	J << x(0), x(1), x(2), 0, 0, 0, -Hx(0) * x(0), -Hx(0) * x(1), -Hx(0) * x(2),
		0, 0, 0, x(0), x(1), x(2), -Hx(1) * x(0), -Hx(1) * x(1), -Hx(1) * x(2);
	J /= w;

	// finally, return the difference between these matrices. The difference should be vanishing
	std::cout << "J: " << std::endl << J << std::endl;
	std::cout << "Finite difference: " << std::endl << difference << std::endl;
	std::cout << J - difference << std::endl;
}

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

		// Normalise each quaternion as these equations only hold on the R4 unit sphere
		ri /= ri.norm();

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
		// TODO is this correct?
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