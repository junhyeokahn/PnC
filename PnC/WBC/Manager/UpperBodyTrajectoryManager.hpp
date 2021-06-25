#pragma once

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/Task.hpp>

class UpperBodyTrajectoryManager {
public:
  UpperBodyTrajectoryManager(Task *_upper_body_task, RobotSystem *_robot);
  virtual ~UpperBodyTrajectoryManager();

  void
  UseNominalUpperBodyJointPos(std::map<std::string, double> _nominal_joint_pos);

private:
  /* data */
  RobotSystem *robot_;
  Task *upper_body_task_;
};
