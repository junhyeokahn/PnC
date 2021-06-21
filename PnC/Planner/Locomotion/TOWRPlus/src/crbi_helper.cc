#include <towr_plus/models/crbi_helper.h>

namespace towr_plus {

CRBIHelper::CRBIHelper(const std::shared_ptr<CompositeRigidBodyInertia> &crbi,
                       const NodeSpline::Ptr &base_linear,
                       const NodeSpline::Ptr &lf_pos_linear,
                       const NodeSpline::Ptr &rf_pos_linear) {
  crbi_ = crbi;
  base_linear_ = base_linear;
  lf_pos_linear_ = lf_pos_linear;
  rf_pos_linear_ = rf_pos_linear;
}

CRBIHelper::~CRBIHelper() {}

CRBIHelper::JacRowMatrix
CRBIHelper::GetDerivativeOfInertiaMatrixWrtBaseLinNodes(double t) {
  JacRowMatrix jac;

  Eigen::Vector3d base_pos = base_linear_->GetPoint(t).p();
  Eigen::Vector3d lf_pos = lf_pos_linear_->GetPoint(t).p();
  Eigen::Vector3d rf_pos = rf_pos_linear_->GetPoint(t).p();

  Eigen::MatrixXd jac_inertia =
      crbi_->ComputeDerivativeWrtInput(base_pos, lf_pos, rf_pos);
  Jacobian jac_inertia_wrt_nodes = jac_inertia.block(0, 0, 6, 3).sparseView() *
                                   base_linear_->GetJacobianWrtNodes(t, kPos);

  jac.at(X).at(X) = jac_inertia_wrt_nodes.row(0);
  jac.at(X).at(Y) = jac_inertia_wrt_nodes.row(3);
  jac.at(X).at(Z) = jac_inertia_wrt_nodes.row(4);

  jac.at(Y).at(X) = jac_inertia_wrt_nodes.row(3);
  jac.at(Y).at(Y) = jac_inertia_wrt_nodes.row(1);
  jac.at(Y).at(Z) = jac_inertia_wrt_nodes.row(5);

  jac.at(Z).at(X) = jac_inertia_wrt_nodes.row(4);
  jac.at(Z).at(Y) = jac_inertia_wrt_nodes.row(5);
  jac.at(Z).at(Z) = jac_inertia_wrt_nodes.row(2);

  return jac;
}

CRBIHelper::JacRowMatrix
CRBIHelper::GetDerivativeOfInertiaMatrixWrtEELinNodes(double t, int ee) {
  JacRowMatrix jac;

  Eigen::Vector3d base_pos = base_linear_->GetPoint(t).p();
  Eigen::Vector3d lf_pos = lf_pos_linear_->GetPoint(t).p();
  Eigen::Vector3d rf_pos = rf_pos_linear_->GetPoint(t).p();

  Eigen::MatrixXd jac_inertia =
      crbi_->ComputeDerivativeWrtInput(base_pos, lf_pos, rf_pos);
  Jacobian jac_inertia_wrt_nodes;
  if (ee == 0) {
    jac_inertia_wrt_nodes = jac_inertia.block(0, 3, 6, 3).sparseView() *
                            lf_pos_linear_->GetJacobianWrtNodes(t, kPos);
  } else if (ee == 1) {
    jac_inertia_wrt_nodes = jac_inertia.block(0, 6, 6, 3).sparseView() *
                            rf_pos_linear_->GetJacobianWrtNodes(t, kPos);

  } else {
  }

  jac.at(X).at(X) = jac_inertia_wrt_nodes.row(0);
  jac.at(X).at(Y) = jac_inertia_wrt_nodes.row(3);
  jac.at(X).at(Z) = jac_inertia_wrt_nodes.row(4);

  jac.at(Y).at(X) = jac_inertia_wrt_nodes.row(3);
  jac.at(Y).at(Y) = jac_inertia_wrt_nodes.row(1);
  jac.at(Y).at(Z) = jac_inertia_wrt_nodes.row(5);

  jac.at(Z).at(X) = jac_inertia_wrt_nodes.row(4);
  jac.at(Z).at(Y) = jac_inertia_wrt_nodes.row(5);
  jac.at(Z).at(Z) = jac_inertia_wrt_nodes.row(2);

  return jac;
}

CRBIHelper::JacRowMatrix
CRBIHelper::GetDerivativeOfInertiaMatrixWrtEEScheduleNodes(double t, int ee) {
  JacRowMatrix jac;

  Eigen::Vector3d base_pos = base_linear_->GetPoint(t).p();
  Eigen::Vector3d lf_pos = lf_pos_linear_->GetPoint(t).p();
  Eigen::Vector3d rf_pos = rf_pos_linear_->GetPoint(t).p();

  Eigen::MatrixXd jac_inertia =
      crbi_->ComputeDerivativeWrtInput(base_pos, lf_pos, rf_pos);
  Jacobian jac_inertia_wrt_nodes;
  if (ee == 0) {
    jac_inertia_wrt_nodes = jac_inertia.block(0, 3, 6, 3).sparseView() *
                            lf_pos_linear_->GetJacobianOfPosWrtDurations(t);
  } else if (ee == 1) {
    jac_inertia_wrt_nodes = jac_inertia.block(0, 6, 6, 3).sparseView() *
                            rf_pos_linear_->GetJacobianOfPosWrtDurations(t);
  } else {
  }

  jac.at(X).at(X) = jac_inertia_wrt_nodes.row(0);
  jac.at(X).at(Y) = jac_inertia_wrt_nodes.row(3);
  jac.at(X).at(Z) = jac_inertia_wrt_nodes.row(4);

  jac.at(Y).at(X) = jac_inertia_wrt_nodes.row(3);
  jac.at(Y).at(Y) = jac_inertia_wrt_nodes.row(1);
  jac.at(Y).at(Z) = jac_inertia_wrt_nodes.row(5);

  jac.at(Z).at(X) = jac_inertia_wrt_nodes.row(4);
  jac.at(Z).at(Y) = jac_inertia_wrt_nodes.row(5);
  jac.at(Z).at(Z) = jac_inertia_wrt_nodes.row(2);

  return jac;
}

CRBIHelper::Jacobian
CRBIHelper::GetDerivativeOfInertiaMatrixWrtBaseLinNodesMult(
    double t, const Eigen::Vector3d &v) {
  JacRowMatrix Id = GetDerivativeOfInertiaMatrixWrtBaseLinNodes(t);
  Jacobian jac = Jacobian(k3D, Id.at(X).at(X).size());

  for (int row : {X, Y, Z}) {
    for (int col : {X, Y, Z}) {
      JacobianRow jac_row = Id.at(row).at(col);
      jac.row(row) += v(col) * jac_row;
    }
  }
  return jac;
}

CRBIHelper::Jacobian CRBIHelper::GetDerivativeOfInertiaMatrixWrtEELinNodesMult(
    double t, const Eigen::Vector3d &v, int ee) {
  JacRowMatrix Id = GetDerivativeOfInertiaMatrixWrtEELinNodes(t, ee);
  Jacobian jac = Jacobian(k3D, Id.at(X).at(X).size());

  for (int row : {X, Y, Z}) {
    for (int col : {X, Y, Z}) {
      JacobianRow jac_row = Id.at(row).at(col);
      jac.row(row) += v(col) * jac_row;
    }
  }
  return jac;
}

CRBIHelper::Jacobian
CRBIHelper::GetDerivativeOfInertiaMatrixWrtEEScheduleNodesMult(
    double t, const Eigen::Vector3d &v, int ee) {
  JacRowMatrix Id = GetDerivativeOfInertiaMatrixWrtEEScheduleNodes(t, ee);
  Jacobian jac = Jacobian(k3D, Id.at(X).at(X).size());

  for (int row : {X, Y, Z}) {
    for (int col : {X, Y, Z}) {
      JacobianRow jac_row = Id.at(row).at(col);
      jac.row(row) += v(col) * jac_row;
    }
  }
  return jac;
}

} // namespace towr_plus
