#pragma once

#include <PnC/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

// Object to manage common trajectory primitives
class UpperBodyJointTrajectoryManager : public TrajectoryManagerBase {
 public:
  UpperBodyJointTrajectoryManager(Task* _upper_body_task, RobotSystem* _robot);
  ~UpperBodyJointTrajectoryManager();
  void paramInitialization(const YAML::Node& node);

  ValkyrieStateProvider* sp_;

  // Use current pose to set the task.
  void initializeRaiseRightArmNow();
  void initializeLowerRightArmNow();

  Task* upper_body_task_;

  Eigen::VectorXd pos_des_;
  Eigen::VectorXd vel_des_;
  Eigen::VectorXd acc_des_;

  Eigen::VectorXd pos_ini_;
  Eigen::VectorXd vel_ini_;
  Eigen::VectorXd acc_ini_;

  double t_start_;
  int rarm_joint_index_;
  double arm_trajectory_duration_;
  double raise_target_value_;
  double lower_target_value_;

  int number_of_joints_;

  bool raising_arm;
  bool lowering_arm;

  // Updates the task desired values
  void getCurrentDesired();
  void computeArmTrajectories();
  void updateDesired();
};
