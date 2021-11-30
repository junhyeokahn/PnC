#pragma once

#include <Eigen/Dense>

#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/whole_body_controllers/task.hpp"

class DracoAngularMomentumTask : public Task {
public:
  DracoAngularMomentumTask(RobotSystem *_robot);

  virtual ~DracoAngularMomentumTask(){};

private:
  void
  update_cmd(Eigen::Matrix3d rot_world_local = Eigen::Matrix3d::Identity());
  void update_jacobian();

  DracoStateProvider *sp_;
};
