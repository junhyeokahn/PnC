#pragma once

#include <vector>

#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/internal_constraint.hpp"
#include "pnc/whole_body_controllers/task.hpp"
#include "utils/util.hpp"

/// class TCIContainer
class TCIContainer {
public:
  /// \{ \name Constructor and Destructor
  TCIContainer(RobotSystem *_robot) { robot_ = _robot; }
  virtual ~TCIContainer() {}
  /// \}

  RobotSystem *robot_;

  /// Task list.
  std::vector<Task *> task_list;

  /// Contact list.
  std::vector<Contact *> contact_list;

  /// InternalConstraint list.
  std::vector<InternalConstraint *> internal_constraint_list;
};
