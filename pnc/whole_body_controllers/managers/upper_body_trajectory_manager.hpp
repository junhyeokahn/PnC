#pragma once

#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/task.hpp"

/// class UpperBodyTrajectoryManager
class UpperBodyTrajectoryManager {
public:
  /// \{ \name Constructor and Destructor
  UpperBodyTrajectoryManager(Task *_upper_body_task, RobotSystem *_robot);
  virtual ~UpperBodyTrajectoryManager();
  /// \}

  /// Use nominal pos as a command.
  void
  UseNominalUpperBodyJointPos(std::map<std::string, double> _nominal_joint_pos);

private:
  RobotSystem *robot_;
  Task *upper_body_task_;
  std::vector<std::string> target_ids_;
};
