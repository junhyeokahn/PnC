#pragma once

#include <string>

#include <Utils/Utilities.hpp>

class FootStepPlanner{
public:
  FootStepPlanner(){}
  virtual ~FootStepPlanner(){}

  virtual void PlannerInitialization(const YAML::Node & node) = 0;
  virtual void getNextFootLocation(const Eigen::Vector3d & com_pos,
                                   const Eigen::Vector3d & com_vel,
                                   Eigen::Vector3d & target_loc,
                                   const void* additional_input = NULL,
                                   void* additional_output = NULL) = 0;
};
