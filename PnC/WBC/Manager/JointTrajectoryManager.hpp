#pragma once

#include <PnC/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <PnC/WBC/BasicTask.hpp>

// Object to manage common trajectory primitives
class JointTrajectoryManager : public TrajectoryManagerBase {
 public:
  JointTrajectoryManager(Task* _joint_task, RobotSystem* _robot);
  ~JointTrajectoryManager(){};

  Task* joint_task_;

  Eigen::VectorXd joint_pos_des_;
  Eigen::VectorXd joint_vel_des_;
  Eigen::VectorXd joint_acc_des_;

  // Updates the task desired values
  void updateDesired();

  // Initialize the joint trajectory
  void initializeJointTrajectory(const double _start_time,
                                 const double _duration,
                                 const Eigen::VectorXd& _target_jpos);
  void updateJointDesired(const double current_time);
  void paramInitialization(const YAML::Node& node){};

  double start_time_;
  double duration_;
  Eigen::VectorXd ini_jpos_;
  Eigen::VectorXd target_jpos_;
};
