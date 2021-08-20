#pragma once

#include <Eigen/Dense>

#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/whole_body_controllers/task.hpp"

class DracoCenterOfMassTask : public Task {
public:
  DracoCenterOfMassTask(RobotSystem *_robot);

  virtual ~DracoCenterOfMassTask(){};

private:
  void update_cmd();
  void update_jacobian();

  DracoStateProvider *sp_;
};
