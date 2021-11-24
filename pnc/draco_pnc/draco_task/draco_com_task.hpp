#pragma once

#include <Eigen/Dense>

#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/filters/digital_filters.hpp"
#include "pnc/whole_body_controllers/task.hpp"
#include "utils/util.hpp"

namespace feedback_source {
constexpr int kCom = 0;
constexpr int kIcp = 1;
} // namespace feedback_source

class DracoCenterOfMassTask : public Task {
public:
  DracoCenterOfMassTask(RobotSystem *_robot, int _feedback_source);

  virtual ~DracoCenterOfMassTask() { delete icp_err_integrator_; };

  Eigen::Vector2d icp_des;
  Eigen::Vector2d icp_dot_des;

private:
  void update_cmd();
  void update_jacobian();

  DracoStateProvider *sp_;

  int feedback_source_;

  ExponentialMovingAverageFilter *icp_err_integrator_;
};
