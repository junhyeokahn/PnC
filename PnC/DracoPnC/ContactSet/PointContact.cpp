#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>

PointContact::PointContact(RobotSystem* robot,
        const std::string & _link_name,
        const double & _mu) : ContactSpec(robot, 3) {
    link_name_ = _link_name;
    max_Fz_ = 1000.;
    mu_ = _mu;
    printf("[[[Point Contact]]] Constructed\n");
}

PointContact::~PointContact(){}

bool PointContact::_UpdateJc(){
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(link_name_);
    Jc_ = Jtmp.block(3, 0, 3, robot_->getNumDofs());
    return true;
}

bool PointContact::_UpdateJcDotQdot(){
    Eigen::VectorXd JcDotQdot_tmp =
        robot_->getBodyNodeCoMJacobianDot(link_name_) * robot_->getQdot();
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    // TODO: we do not consider local frame rotation acceleration
    JcDotQdot_.setZero();
    return true;
}

bool PointContact::_UpdateUf(){
    Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(3, 3);
    rot = (robot_->getBodyNodeCoMIsometry(link_name_).linear()).transpose();

    Uf_ = Eigen::MatrixXd::Zero(6, dim_contact_);
    // Fx(0), Fy(1), Fz(2)

    // Linear
    Uf_(0, 2) = 1.;  // Fz >= 0

    Uf_(1, 0) = 1.; Uf_(1, 2) = mu_;
    Uf_(2, 0) = -1.; Uf_(2, 2) = mu_;

    Uf_(3, 1) = 1.; Uf_(3, 2) = mu_;
    Uf_(4, 1) = -1.; Uf_(4, 2) = mu_;

    // Upper bound of vertical directional reaction force
    Uf_(5, 2) = -1.;  // -Fz >= -max_Fz_

    Uf_ *= rot;
    return true;
}

bool PointContact::_UpdateInequalityVector(){
    ieq_vec_ = Eigen::VectorXd::Zero(6);
    ieq_vec_[5] = -max_Fz_;
    return true;
}

bool PointContact::_UpdateContactGeometry() {
    return true;
}
