#pragma once

#include <Eigen/Dense>

#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/whole_body_controllers/task.hpp"

class DracoAngularMomentumTask : public Task {
public:
  DracoAngularMomentumTask(RobotSystem *_robot);

  virtual ~DracoAngularMomentumTask(){};

private:
  void update_cmd();
  void update_jacobian();

  DracoStateProvider *sp_;
};
