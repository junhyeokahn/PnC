#pragma once

#include <towr_plus/models/composite_rigid_body_inertia.h>
#include <towr_plus/variables/cartesian_dimensions.h>
#include <towr_plus/variables/node_spline.h>

namespace towr_plus {

class CRBIHelper {
public:
  using JacobianRow = Eigen::SparseVector<double, Eigen::RowMajor>;
  using JacRowMatrix = std::array<std::array<JacobianRow, k3D>, k3D>;
  using MatrixSXd = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using Jacobian = MatrixSXd;

  CRBIHelper(const std::shared_ptr<CompositeRigidBodyInertia> &crbi,
             const NodeSpline::Ptr &base_linear,
             const NodeSpline::Ptr &lf_pos_linear,
             const NodeSpline::Ptr &rf_pos_linear);
  virtual ~CRBIHelper();

  JacRowMatrix GetDerivativeOfInertiaMatrixWrtBaseLinNodes(double t);
  JacRowMatrix GetDerivativeOfInertiaMatrixWrtEELinNodes(double t, int ee);
  JacRowMatrix GetDerivativeOfInertiaMatrixWrtEEScheduleNodes(double t, int ee);

  Jacobian
  GetDerivativeOfInertiaMatrixWrtBaseLinNodesMult(double t,
                                                  const Eigen::Vector3d &v);
  Jacobian GetDerivativeOfInertiaMatrixWrtEELinNodesMult(
      double t, const Eigen::Vector3d &v, int ee);
  Jacobian GetDerivativeOfInertiaMatrixWrtEEScheduleNodesMult(
      double t, const Eigen::Vector3d &v, int ee);

private:
  /* data */
  std::shared_ptr<CompositeRigidBodyInertia> crbi_;
  NodeSpline::Ptr base_linear_;
  NodeSpline::Ptr lf_pos_linear_;
  NodeSpline::Ptr rf_pos_linear_;
};

} // namespace towr_plus
