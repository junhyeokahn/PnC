#include <PnC/WBC/BasicContactSpec.hpp>

PointContactSpec::PointContactSpec(RobotSystem* robot,
        const std::string & _link_name,
        const double & _mu) : ContactSpec(robot, 3) {
    myUtils::pretty_constructor(3, _link_name+" Point Contact");

    link_name_ = _link_name;
    max_Fz_ = 1000.;
    mu_ = _mu;
}

PointContactSpec::~PointContactSpec(){}

bool PointContactSpec::_UpdateJc(){
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(link_name_, robot_->getBodyNode(link_name_));
    Jc_ = Jtmp.block(3, 0, 3, robot_->getNumDofs());
    return true;
}

bool PointContactSpec::_UpdateJcDotQdot(){
    Eigen::VectorXd JcDotQdot_tmp =
        robot_->getBodyNodeCoMJacobianDot(link_name_, robot_->getBodyNode(link_name_)) * robot_->getQdot();
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    JcDotQdot_.setZero();
    return true;
}

bool PointContactSpec::_UpdateUf(){
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

bool PointContactSpec::_UpdateInequalityVector(){
    ieq_vec_ = Eigen::VectorXd::Zero(6);
    ieq_vec_[5] = -max_Fz_;
    return true;
}

FixedBodyContactSpec::FixedBodyContactSpec(RobotSystem* _robot) : ContactSpec(_robot, 6)
{
    myUtils::pretty_constructor(3, "Fixed Body Contact");

    Jc_ = Eigen::MatrixXd::Zero(dim_contact_, robot_->getNumDofs());
}

FixedBodyContactSpec::~FixedBodyContactSpec(){ }

bool FixedBodyContactSpec::_UpdateJc(){
    for(int i(0);i<dim_contact_; ++i)  Jc_(i,i) = 1.;
    return true;
}

bool FixedBodyContactSpec::_UpdateJcDotQdot(){
    JcDotQdot_ = Eigen::VectorXd::Zero(dim_contact_);
    return true;
}

bool FixedBodyContactSpec::_UpdateUf(){
    Uf_ = Eigen::MatrixXd::Zero(1, dim_contact_);
    return true;
}

bool FixedBodyContactSpec::_UpdateInequalityVector(){
    ieq_vec_ = Eigen::VectorXd::Zero(1);
    return true;
}
