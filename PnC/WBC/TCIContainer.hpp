#pragma once

#include <vector>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <utils/util.hpp>

#include <PnC/WBC/Contact.hpp>
#include <PnC/WBC/InternalConstraint.hpp>
#include <PnC/WBC/Task.hpp>

class TCIContainer {
public:
  TCIContainer(RobotSystem *_robot) { robot_ = _robot; }
  virtual ~TCIContainer() {}

  RobotSystem *robot_;
  std::vector<Task *> task_list;
  std::vector<Contact *> contact_list;
  std::vector<InternalConstraint *> internal_constraint_list;
};
