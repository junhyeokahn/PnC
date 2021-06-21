#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class BasicTask : public Task {
public:
  enum Type {
    JOINT,
    SELECTED_JOINT,
    LINK_XYZ,
    ISOLATED_LINK_XYZ,
    LINK_ORI,
    ISOLATED_LINK_ORI,
    COM,
    ISOLATED_COM,
  };

  BasicTask(RobotSystem *robot_, const Type taskType_, const int _dim,
            std::vector<std::string> _target_id = std::vector<std::string>());
  virtual ~BasicTask(){};

private:
  void update_cmd();
  void update_jacobian();

  Type task_type_;
};
