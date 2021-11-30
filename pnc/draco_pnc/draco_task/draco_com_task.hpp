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

namespace feedback_height_target {
constexpr int kComHeight = 0;
constexpr int kBaseHeight = 1;
} // namespace feedback_height_target

class DracoCenterOfMassTask : public Task {
public:
  DracoCenterOfMassTask(RobotSystem *_robot, int _feedback_source,
                        int _feedback_height_target);

  virtual ~DracoCenterOfMassTask() { delete icp_err_integrator_; };

  Eigen::Vector2d icp_des;
  Eigen::Vector2d icp_dot_des;

private:
  void
  update_cmd(Eigen::Matrix3d rot_world_local = Eigen::Matrix3d::Identity());
  void update_jacobian();

  DracoStateProvider *sp_;

  int feedback_source_;
  int feedback_height_target_;

  ExponentialMovingAverageFilter *icp_err_integrator_;
};
