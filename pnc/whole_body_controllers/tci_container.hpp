#pragma once

#include <vector>

#include <pnc/robot_system/robot_system.hpp>
#include <utils/util.hpp>

#include <pnc/whole_body_controllers/contact.hpp>
#include <pnc/whole_body_controllers/internal_constraint.hpp>
#include <pnc/whole_body_controllers/task.hpp>

class TCIContainer {
public:
  TCIContainer(RobotSystem *_robot) { robot_ = _robot; }
  virtual ~TCIContainer() {}

  RobotSystem *robot_;
  std::vector<Task *> task_list;
  std::vector<Contact *> contact_list;
  std::vector<InternalConstraint *> internal_constraint_list;
};
