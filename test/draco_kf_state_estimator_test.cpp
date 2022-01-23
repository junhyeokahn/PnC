#include <gtest/gtest.h>

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"
#include "pnc/draco_pnc/draco_kf_state_estimator.hpp"

TEST(DracoKFStateEstimatorTest, standingStraight)
{
  DartRobotSystem *draco = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf",
                                               false, false);
  DracoKFStateEstimator estimator = DracoKFStateEstimator(draco);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}