#include "gtest/gtest.h"

#include <Configuration.h>
#include <Eigen/Dense>
#include <Utils/IO/IOUtilities.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/IHWBC/IHWBC_JointIntegrator.hpp>

TEST(IHWBC_JOINT_INTEGRATION, constructor_test) {
	int num_joints = 10;
	double dt = 1e-3;
	IHWBC_JointIntegrator joint_integrator(num_joints, dt);

	// Leaky Integrator frequency. Much lower than usual cut-offs.
	joint_integrator.setVelocityFrequencyCutOff(2.0);
	joint_integrator.setPositionFrequencyCutOff(1.0);
	joint_integrator.setMaxPositionError(0.2);
	joint_integrator.printIntegrationParams();

	// Initialize current velocity and positions
	Eigen::VectorXd vel = Eigen::VectorXd::Zero(num_joints);
	Eigen::VectorXd pos = M_PI/4.0*Eigen::VectorXd::Ones(num_joints);

	joint_integrator.initializeStates(vel, pos);

	// Initialize output containers
	Eigen::VectorXd vel_out = Eigen::VectorXd::Zero(num_joints);
	Eigen::VectorXd pos_out = Eigen::VectorXd::Zero(num_joints);

	// Initialize Output variables
	// Set joint acceleration
	Eigen::VectorXd acc = 2.0*Eigen::VectorXd::Ones(num_joints);

	myUtils::pretty_print(acc, std::cout, "acc");
	myUtils::pretty_print(vel, std::cout, "vel");
	myUtils::pretty_print(pos, std::cout, "pos");

	joint_integrator.integrate(acc, vel, pos, vel_out, pos_out);

	myUtils::pretty_print(vel_out, std::cout, "vel_out");
	myUtils::pretty_print(pos_out, std::cout, "pos_out");

}