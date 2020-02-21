#ifndef ALM_HERMITE_QUATERNION_CURVE_H
#define ALM_HERMITE_QUATERNION_CURVE_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <algorithm>

class HermiteQuaternionCurve{
public:
	HermiteQuaternionCurve(const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
						   const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end);
	~HermiteQuaternionCurve();

	// All values are expressed in "world frame"
	void evaluate(const double & s_in, Eigen::Quaterniond & quat_out);
	void getAngularVelocity(const double & s_in, Eigen::Vector3d & ang_vel_out);
	void getAngularAcceleration(const double & s_in, Eigen::Vector3d & ang_acc_out);

private:
	Eigen::Quaterniond qa; // Starting quaternion
	Eigen::Vector3d omega_a; // Starting Angular Velocity
	Eigen::Quaterniond qb; // Ending quaternion
	Eigen::Vector3d omega_b; // Ending Angular velocity

	Eigen::AngleAxisd omega_a_aa; // axis angle representation of omega_a
	Eigen::AngleAxisd omega_b_aa; // axis angle representation of omega_b

	void initialize_data_structures();

	void computeBasis(const double & s_in); // computes the basis functions
	void computeOmegas();

	Eigen::Quaterniond q0; // quat0
	Eigen::Quaterniond q1; // quat1
	Eigen::Quaterniond q2; // quat1
	Eigen::Quaterniond q3; // quat1

	double b1; // basis 1
	double b2; // basis 2
	double b3; // basis 3

	double bdot1; // 1st derivative of basis 1
	double bdot2; // 1st derivative of basis 2
	double bdot3; // 1st derivative of basis 3

	double bddot1; // 2nd derivative of basis 1
	double bddot2; // 2nd derivative of basis 2
	double bddot3; // 2nd derivative of basis 3

	Eigen::Vector3d omega_1;
	Eigen::Vector3d omega_2;
	Eigen::Vector3d omega_3;

	Eigen::AngleAxisd omega_1aa;
	Eigen::AngleAxisd omega_2aa;
	Eigen::AngleAxisd omega_3aa;

	// Allocate memory for quaternion operations
	Eigen::Quaterniond qtmp1;
	Eigen::Quaterniond qtmp2;
	Eigen::Quaterniond qtmp3;

	// progression variable
	double s_;
	// by default clamps within 0 and 1.
	double clamp(const double & s_in, double lo = 0.0, double hi = 1.0);

	void printQuat(const Eigen::Quaterniond & quat);

};

#endif