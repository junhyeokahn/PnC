#include <PnC/WBC/BasicContactSpec.hpp>

PointContactSpec::PointContactSpec(RobotSystem* robot, int _link_idx,
                                   double _mu)
    : ContactSpec(robot, 3) {
    myUtils::pretty_constructor(3, "Point Contact Spec");

    link_idx_ = _link_idx;
    max_Fz_ = 500.;
    mu_ = _mu;
}

PointContactSpec::~PointContactSpec() {}

bool PointContactSpec::_UpdateJc() {
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(link_idx_);
    Jc_ = Jtmp.block(dim_contact_, 0, dim_contact_, robot_->getNumDofs());
    return true;
}

bool PointContactSpec::_UpdateJcDotQdot() {
    Eigen::VectorXd JcDotQdot_tmp =
        robot_->getBodyNodeCoMJacobianDot(link_idx_,
                                          robot_->getBodyNode(link_idx_)) *
        robot_->getQdot();
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    //JcDotQdot_.setZero();
    return true;
}

bool PointContactSpec::_UpdateUf() {
    Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(dim_contact_, dim_contact_);
    rot = (robot_->getBodyNodeCoMIsometry(link_idx_).linear()).transpose();

    Uf_ = Eigen::MatrixXd::Zero(6, dim_contact_);
    // Fx(0), Fy(1), Fz(2)

    // Linear
    Uf_(0, 2) = 1.;  // Fz >= 0

    Uf_(1, 0) = 1.;
    Uf_(1, 2) = mu_;
    Uf_(2, 0) = -1.;
    Uf_(2, 2) = mu_;

    Uf_(3, 1) = 1.;
    Uf_(3, 2) = mu_;
    Uf_(4, 1) = -1.;
    Uf_(4, 2) = mu_;

    // Upper bound of vertical directional reaction force
    Uf_(5, 2) = -1.;  // -Fz >= -max_Fz_

    // Uf_ *= rot;
    return true;
}

bool PointContactSpec::_UpdateInequalityVector() {
    ieq_vec_ = Eigen::VectorXd::Zero(6);
    ieq_vec_[5] = -max_Fz_;
    return true;
}

FixedBodyContactSpec::FixedBodyContactSpec(RobotSystem* _robot)
    : ContactSpec(_robot, 6) {
    myUtils::pretty_constructor(3, "Fixed Body Contact");

    Jc_ = Eigen::MatrixXd::Zero(dim_contact_, robot_->getNumDofs());
}

FixedBodyContactSpec::~FixedBodyContactSpec() {}

bool FixedBodyContactSpec::_UpdateJc() {
    for (int i(0); i < dim_contact_; ++i) Jc_(i, i) = 1.;
    return true;
}

bool FixedBodyContactSpec::_UpdateJcDotQdot() {
    JcDotQdot_ = Eigen::VectorXd::Zero(dim_contact_);
    return true;
}

bool FixedBodyContactSpec::_UpdateUf() {
    Uf_ = Eigen::MatrixXd::Zero(1, dim_contact_);
    return true;
}

bool FixedBodyContactSpec::_UpdateInequalityVector() {
    ieq_vec_ = Eigen::VectorXd::Zero(1);
    return true;
}

SurfaceContactSpec::SurfaceContactSpec(RobotSystem* robot, int _link_idx,
                                       double _x, double _y, double _mu)
    : ContactSpec(robot, 6) {
    myUtils::pretty_constructor(3, "Surface Contact Spec");

    link_idx_ = _link_idx;
    max_Fz_ = 1500.;
    mu_ = _mu;
    x_ = _x;
    y_ = _y;
}

SurfaceContactSpec::~SurfaceContactSpec() {}

bool SurfaceContactSpec::_UpdateJc() {
    Jc_ = robot_->getBodyNodeJacobian(link_idx_);
    return true;
}

bool SurfaceContactSpec::_UpdateJcDotQdot() {
    JcDotQdot_ = robot_->getBodyNodeJacobianDot(link_idx_) * robot_->getQdot();
    return true;
}

bool SurfaceContactSpec::_UpdateUf() {
    Uf_ = Eigen::MatrixXd::Zero(16 + 2, dim_contact_);

    Eigen::MatrixXd U;
    _setU(x_, y_, mu_, U);
    Eigen::MatrixXd Rot_foot_mtx =
        robot_->getBodyNodeIsometry(link_idx_).linear();
    Eigen::MatrixXd Rot_foot(6, 6);
    Rot_foot.setZero();
    Rot_foot.block(0, 0, 3, 3) = Rot_foot_mtx.transpose();
    Rot_foot.block(3, 3, 3, 3) = Rot_foot_mtx.transpose();

    // Contact Wrench transform as discussed in:
    // https://github.com/stephane-caron/analytical-wrench-cone/issues/2
    // Uf_ = U * Rot_foot;

    // Equivalent to Adjoint mapping below:
    // With the geometric Jacobian used by dart (world frame representation of body twists and wrenches),
    // The wrenches are defined to be at link frame center (p = 0) with moment axes parallel to the world frame.
    Eigen::MatrixXd Adj_foot(6, 6); Adj_foot.setZero();
    Eigen::VectorXd pos_vec  = Eigen::Vector3d::Zero();
    Adj_foot = myUtils::Adjoint(Rot_foot_mtx, pos_vec);
    Uf_ = U * Adj_foot.transpose();

    return true;
}

bool SurfaceContactSpec::_UpdateInequalityVector() {
    ieq_vec_ = Eigen::VectorXd::Zero(16 + 2);
    ieq_vec_[17] = -max_Fz_;
    return true;
}

void SurfaceContactSpec::_setU(double x, double y, double mu,
                               Eigen::MatrixXd& U) {
    U = Eigen::MatrixXd::Zero(16 + 2, 6);

    U(0, 5) = 1.;

    U(1, 3) = 1.;
    U(1, 5) = mu;
    U(2, 3) = -1.;
    U(2, 5) = mu;

    U(3, 4) = 1.;
    U(3, 5) = mu;
    U(4, 4) = -1.;
    U(4, 5) = mu;

    U(5, 0) = 1.;
    U(5, 5) = y;
    U(6, 0) = -1.;
    U(6, 5) = y;

    U(7, 1) = 1.;
    U(7, 5) = x;
    U(8, 1) = -1.;
    U(8, 5) = x;

    // Tau
    U(9, 0) = -mu;
    U(9, 1) = -mu;
    U(9, 2) = 1;
    U(9, 3) = y;
    U(9, 4) = x;
    U(9, 5) = (x + y) * mu;

    U(10, 0) = -mu;
    U(10, 1) = mu;
    U(10, 2) = 1;
    U(10, 3) = y;
    U(10, 4) = -x;
    U(10, 5) = (x + y) * mu;

    U(11, 0) = mu;
    U(11, 1) = -mu;
    U(11, 2) = 1;
    U(11, 3) = -y;
    U(11, 4) = x;
    U(11, 5) = (x + y) * mu;

    U(12, 0) = mu;
    U(12, 1) = mu;
    U(12, 2) = 1;
    U(12, 3) = -y;
    U(12, 4) = -x;
    U(12, 5) = (x + y) * mu;
    /////////////////////////////////////////////////
    U(13, 0) = -mu;
    U(13, 1) = -mu;
    U(13, 2) = -1;
    U(13, 3) = -y;
    U(13, 4) = -x;
    U(13, 5) = (x + y) * mu;

    U(14, 0) = -mu;
    U(14, 1) = mu;
    U(14, 2) = -1;
    U(14, 3) = -y;
    U(14, 4) = x;
    U(14, 5) = (x + y) * mu;

    U(15, 0) = mu;
    U(15, 1) = -mu;
    U(15, 2) = -1;
    U(15, 3) = y;
    U(15, 4) = -x;
    U(15, 5) = (x + y) * mu;

    U(16, 0) = mu;
    U(16, 1) = mu;
    U(16, 2) = -1;
    U(16, 3) = y;
    U(16, 4) = x;
    U(16, 5) = (x + y) * mu;
    // ////////////////////////////////////////////////////
    U(17, 5) = -1.;
}
