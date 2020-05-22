#pragma once

#include <vector>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>

#include <PnC/WBC/Task.hpp>
#include <PnC/WBC/ContactSpec.hpp>

// Simple class to hold on to task list and contact list

class TaskAndForceContainer {
public:
  TaskAndForceContainer(RobotSystem* _robot){
  	robot_ = _robot;
  }
  virtual ~TaskAndForceContainer(){}
  virtual void paramInitialization(const YAML::Node& node) = 0;

  RobotSystem* robot_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;
};