#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>

LineContact::LineContact(RobotSystem* robot,
                         const std::string & _link_name,
                         const double & _mu,
                         const double & _gamma) : ContactSpec(robot, 5) {
    link_name_ = _link_name;
    max_Fz_ = 1000.;
    mu_ = _mu;
    gamma_ = _gamma;
}

LineContact::~LineContact(){}

bool LineContact::_UpdateJc(){
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(link_name_);
    Jc_ = Jtmp.block(1, 0, 5, robot_->getNumDofs());
    return true;
}

bool LineContact::_UpdateJcDotQdot(){
    Eigen::VectorXd JcDotQdot_tmp =
        robot_->getBodyNodeCoMJacobianDot(link_name_) * robot_->getQdot();
    JcDotQdot_ = JcDotQdot_tmp.tail(dim_contact_);

    JcDotQdot_.setZero();
    return true;
}

bool LineContact::_UpdateUf(){
    Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(3, 3);
    rot = (robot_->getBodyNodeCoMIsometry(link_name_).linear()).transpose();
    Eigen::MatrixXd aug_rot = Eigen::MatrixXd::Zero(6, 5);
    aug_rot.block(0, 0, 3, 2) = rot.block(0, 1, 3, 2);
    aug_rot.block(3, 2, 3, 3) = rot;

    Uf_ = Eigen::MatrixXd::Zero(10, dim_contact_);

    /*
     * Reference : https://github.com/ayonga/frost-dev/search?utf8=%E2%9C%93&q=LineContactWithFriction&type=
     * Wr = [Ty(0), Tz(1), Fx(2), Fy(3), Fz(4)]
     * la = center to back, lb = center to front
     */
    double la(0.75); double lb(0.75);

    // Fz >= 0
    Uf_(0, 4) = 1.;

    // Friction Cone
    Uf_(1, 2) =  1.; Uf_(1, 4) = mu_;
    Uf_(2, 2) = -1.; Uf_(2, 4) = mu_;

    Uf_(3, 3) =  1.; Uf_(3, 4) = mu_;
    Uf_(4, 3) = -1.; Uf_(4, 4) = mu_;

    Uf_(5, 1) =  1.; Uf_(5, 4) = gamma_;
    Uf_(6, 1) = -1.; Uf_(6, 4) = gamma_;

    // ZMP
    Uf_(7, 0) = -1; Uf_(7, 4) = la;
    Uf_(8, 0) =  1; Uf_(8, 4) = lb;

    // -Fz >= -max_Fz_
    Uf_(9, 4) = -1.;

    Uf_ *= aug_rot;
    return true;
}

bool LineContact::_UpdateInequalityVector(){
    ieq_vec_ = Eigen::VectorXd::Zero(10);
    ieq_vec_[9] = -max_Fz_;
    return true;
}

bool LineContact::_UpdateContactGeometry(){
    return true;
}
