#pragma once

#include <vector>

#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/internal_constraint.hpp"
#include "pnc/whole_body_controllers/task.hpp"
#include "utils/util.hpp"

class KinWBC {
public:
  KinWBC(const Eigen::MatrixXd &_sf);
  ~KinWBC() {}

  bool solve(const Eigen::VectorXd &curr_jpos,
             const std::vector<Task *> &task_list,
             const std::vector<Contact *> &contact_list,
             const std::vector<InternalConstraint *> &_internal_constraint_list,
             Eigen::VectorXd &jpos_cmd, Eigen::VectorXd &jvel_cmd,
             Eigen::VectorXd &jacc_cmd);

private:
  void _PseudoInverse(const Eigen::MatrixXd J, Eigen::MatrixXd &Jinv);
  void _BuildProjectionMatrix(const Eigen::MatrixXd &J, Eigen::MatrixXd &N);

  double threshold_;
  int num_qdot_;
  int num_act_joint_;

  int n_floating_;
  int n_qdot_;
};
