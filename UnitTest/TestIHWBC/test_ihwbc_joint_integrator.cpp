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

	joint_integrator.setVelocityFrequencyCutOff(2.0);
	joint_integrator.setPositionFrequencyCutOff(1.0);
	joint_integrator.printIntegrationParams();
}