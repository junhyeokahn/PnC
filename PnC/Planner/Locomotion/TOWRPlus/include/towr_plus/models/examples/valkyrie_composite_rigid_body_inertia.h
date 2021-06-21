#pragma once

#include <towr_plus/models/composite_rigid_body_inertia.h>
#include <towr_plus/models/examples/valkyrie_crbi_helper.h>

namespace towr_plus {

class ValkyrieCompositeRigidBodyInertia : public CompositeRigidBodyInertia {
public:
  ValkyrieCompositeRigidBodyInertia();
  virtual ~ValkyrieCompositeRigidBodyInertia();

  Eigen::MatrixXd ComputeInertia(const Eigen::Vector3d &base_pos,
                                 const Eigen::Vector3d &lf_pos,
                                 const Eigen::Vector3d &rf_pos);

  Eigen::MatrixXd ComputeDerivativeWrtInput(const Eigen::Vector3d &base_pos,
                                            const Eigen::Vector3d &lf_pos,
                                            const Eigen::Vector3d &rf_pos);

private:
  casadi_int f_sz_arg_;
  casadi_int f_sz_res_;
  casadi_int f_sz_iw_;
  casadi_int f_sz_w_;
  casadi_int jac_f_sz_arg_;
  casadi_int jac_f_sz_res_;
  casadi_int jac_f_sz_iw_;
  casadi_int jac_f_sz_w_;

  double **f_x_;
  double **f_y_;
  double **jac_f_x_;
  double **jac_f_y_;

  Eigen::MatrixXd f_in_ph_;
  Eigen::MatrixXd f_out_ph_;
  Eigen::MatrixXd jac_f_out_ph_;
};

} // namespace towr_plus
