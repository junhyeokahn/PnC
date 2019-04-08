#include <PnC/FixedAtlasPnC/CtrlSet/OSCOriCtrl.hpp>
#include <PnC/FixedAtlasPnC/FixedAtlasInterface.hpp>
#include <PnC/FixedAtlasPnC/FixedAtlasDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

OSCOriCtrl::OSCOriCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(3, "OSC ORI Ctrl");
    ctrl_count_ = 0;
}

OSCOriCtrl::~OSCOriCtrl() {}

void OSCOriCtrl::oneStep(void* _cmd) {
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumDofs());
    Eigen::Vector3d so3_delta = Eigen::Vector3d::Zero();
    Eigen::Vector3d curr_so3_des = Eigen::Vector3d::Zero();
    Eigen::Quaternion<double> quat_delta;
    Eigen::Quaternion<double> curr_quat_des;

    Eigen::Quaternion<double> act_pos_quat_;
    act_pos_quat_= robot_->getBodyNodeCoMIsometry("head").linear();
    Eigen::Vector3d act_vel_so3 = robot_->getBodyNodeCoMSpatialVelocity("head").head(3); 
    double t = myUtils::smooth_changing(0, 1,end_time_, state_machine_time_);
    double tdot = myUtils::smooth_changing_vel(0, 1,end_time_, state_machine_time_);
  
    so3_delta = so3_ori_error * t;
    quat_delta = dart::math::expToQuat(so3_delta);
    curr_quat_des = quat_delta * ini_pos_quat_;

    curr_so3_des = so3_ori_error * tdot;

    Eigen::Vector3d head_acc_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d ori_error = Eigen::Vector3d::Zero();
    ori_error = dart::math::quatToExp(curr_quat_des * (act_pos_quat_.inverse()));
    Eigen::Vector3d head_acc_ff = Eigen::Vector3d::Zero();

    for (int i = 0; i < 3; ++i) {
        head_acc_des[i] =
            head_acc_ff[i] + head_kp_[i] * ori_error[i] 
            + head_kd_[i] * (curr_so3_des[i] - act_vel_so3[i]);
    }
    //std::cout << head_acc_des[0] << std::endl;
    //std::cout << head_acc_des[1] << std::endl;
    //std::cout << head_acc_des[2] << std::endl;
    Eigen::MatrixXd J_head = robot_->getBodyNodeJacobian("head").block(0,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_head_bar = Eigen::MatrixXd::Zero(robot_->getNumDofs(),3);
    myUtils::weightedInverse(J_head,robot_->getInvMassMatrix(),J_head_bar);
    Eigen::MatrixXd J_head_dot = robot_->getBodyNodeJacobianDot("head").block(0,0,3,robot_->getNumDofs());
    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(robot_->getNumDofs());

    qddot_des = J_head_bar * (head_acc_des - J_head_dot*(robot_->getQdot()));

    gamma = robot_->getMassMatrix() * qddot_des + robot_->getCoriolisGravity();

    ((FixedAtlasCommand*)_cmd)->jtrq = gamma;
    //std::cout << ctrl_count_ << std::endl;
    ++ctrl_count_;
    state_machine_time_= ctrl_count_ * AtlasAux::ServoRate;
    std::cout << state_machine_time_ << std::endl;
}

void OSCOriCtrl::firstVisit() {
    state_machine_time_= 0.;
    ctrl_count_ = 0;
    ini_pos_quat_ = robot_->getBodyNodeCoMIsometry("head").linear();
    fin_pos_quat_ = Eigen::Quaternion<double>(target_pos_[0],target_pos_[1],target_pos_[2],target_pos_[3]); 
    quat_ori_error = fin_pos_quat_*(ini_pos_quat_.inverse());
    so3_ori_error = dart::math::quatToExp(quat_ori_error);
}
void OSCOriCtrl::lastVisit() {
}

bool OSCOriCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void OSCOriCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
        myUtils::readParameter(node, "head_kp", head_kp_);
        myUtils::readParameter(node, "head_kd", head_kd_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
