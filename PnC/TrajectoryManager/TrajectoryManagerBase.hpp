#pragma once

#include <vector>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>

#include <PnC/WBC/ContactSpec.hpp>
#include <PnC/WBC/Task.hpp>

// Object to manage common trajectory primitives.
// Base class for the trajectory manager

class TrajectoryManagerBase {
 public:
  TrajectoryManagerBase(RobotSystem* _robot) { robot_ = _robot; }
  virtual ~TrajectoryManagerBase() {}
  virtual void paramInitialization(const YAML::Node& node) = 0;

  RobotSystem* robot_;

  double traj_start_time_;
  double traj_end_time_;
};
