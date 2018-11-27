#include <PnC/WBC/RectangularContactSpec.hpp>

RectangularContactSpec::RectangularContactSpec(RobotSystem* _robot,
                                           const std::string & _link_name,
                                           const double & _mu ) : ContactSpec(_robot, 6) {

    mu_ = _mu;
    Uf_ = Eigen::MatrixXd::Zero(18, dim_contact_);
    ieq_vec_ = Eigen::VectorXd::Zero(18);
    max_Fz_ = 1000.;
    link_name_ = _link_name;
    box_size_ = robot_->getBodyNodeCollisionShape(_link_name);

    printf("[%s Rectangular Contact Spec] is Constructed\n", link_name_.c_str());
}

bool RectangularContactSpec::_UpdateContactGeometry() {
    iso_world_to_contact_center_ = robot_->getBodyNodeCollisionIsometry(link_name_);
    Eigen::VectorXd tmp = iso_world_to_contact_center_.translation();
    vec_bodynode_to_contact_surface_ = robot_->getBodyNodeCollisionIsometry(link_name_, robot_->getBodyNode(link_name_)).translation();
    vec_bodynode_to_contact_surface_[2] -= box_size_[2]/2;
    return true;
}

bool RectangularContactSpec::_UpdateJc() {
    Jc_ =
        robot_->getBodyNodeJacobian(link_name_, vec_bodynode_to_contact_surface_);
    return true;
}

bool RectangularContactSpec::_UpdateJcDotQdot() {
    JcDotQdot_ =
        robot_->getBodyNodeJacobianDot(link_name_, vec_bodynode_to_contact_surface_) *
        robot_->getQdot();
    //JcDotQdot_.setZero();
    return true;
}

bool RectangularContactSpec::_UpdateUf() {

    double x(box_size_[0]/2); double y(box_size_[1]/2);
    Eigen::MatrixXd aug_rot = Eigen::MatrixXd::Zero(6, 6);
    aug_rot.block(0, 0, 3, 3) = (iso_world_to_contact_center_.linear()).transpose();
    aug_rot.block(3, 3, 3, 3) = (iso_world_to_contact_center_.linear()).transpose();

    Uf_.setZero();
    Uf_(0, 5) = 1.;

    Uf_(1, 3) = 1. ; Uf_(1, 5) = mu_;
    Uf_(2, 3) = -1.; Uf_(2, 5) = mu_;

    Uf_(3, 4) = 1. ; Uf_(3, 5) = mu_;
    Uf_(4, 4) = -1.; Uf_(4, 5) = mu_;

    Uf_(5, 0) = 1. ; Uf_(5, 5) = y;
    Uf_(6, 0) = -1.; Uf_(6, 5) = y;

    Uf_(7, 1) = 1. ; Uf_(7, 5) = x;
    Uf_(8, 1) = -1.; Uf_(8, 5) = x;

    // Tau
    Uf_(9, 0) = -mu_; Uf_(9, 1) = -mu_; Uf_(9, 2) = 1;
    Uf_(9, 3) = y;    Uf_(9, 4) = x;    Uf_(9, 5) = (x + y)*mu_;

    Uf_(10, 0) = -mu_; Uf_(10, 1) = mu_; Uf_(10, 2) = 1;
    Uf_(10, 3) = y;    Uf_(10, 4) = -x;  Uf_(10, 5) = (x + y)*mu_;

    Uf_(11, 0) = mu_; Uf_(11, 1) = -mu_; Uf_(11, 2) = 1;
    Uf_(11, 3) = -y;  Uf_(11, 4) = x;    Uf_(11, 5) = (x + y)*mu_;

    Uf_(12, 0) = mu_; Uf_(12, 1) = mu_; Uf_(12, 2) = 1;
    Uf_(12, 3) = -y;  Uf_(12, 4) = -x;  Uf_(12, 5) = (x + y)*mu_;
    /////////////////////////////////////////////////
    Uf_(13, 0) = -mu_; Uf_(13, 1) = -mu_; Uf_(13, 2) = -1;
    Uf_(13, 3) = -y;   Uf_(13, 4) = -x;   Uf_(13, 5) = (x + y)*mu_;

    Uf_(14, 0) = -mu_; Uf_(14, 1) = mu_; Uf_(14, 2) = -1;
    Uf_(14, 3) = -y;   Uf_(14, 4) = x;   Uf_(14, 5) = (x + y)*mu_;

    Uf_(15, 0) = mu_; Uf_(15, 1) = -mu_; Uf_(15, 2) = -1;
    Uf_(15, 3) = y;   Uf_(15, 4) = -x;   Uf_(15, 5) = (x + y)*mu_;

    Uf_(16, 0) = mu_; Uf_(16, 1) = mu_; Uf_(16, 2) = -1;
    Uf_(16, 3) = y;   Uf_(16, 4) = x;   Uf_(16, 5) = (x + y)*mu_;
    // ////////////////////////////////////////////////////
    Uf_(17,5) = -1.;

    Uf_ *= aug_rot;

    return true;
}

bool RectangularContactSpec::_UpdateInequalityVector() {
    ieq_vec_ = Eigen::VectorXd::Zero(18);
    ieq_vec_[17] = -max_Fz_;

    return true;
}
