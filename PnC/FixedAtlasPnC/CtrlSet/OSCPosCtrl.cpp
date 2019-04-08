#include <PnC/FixedAtlasPnC/CtrlSet/OSCPosCtrl.hpp>
#include <PnC/FixedAtlasPnC/FixedAtlasInterface.hpp>
#include <PnC/FixedAtlasPnC/FixedAtlasDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

OSCPosCtrl::OSCPosCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "OSC POS Ctrl");
    ctrl_count_ = 0;
}

OSCPosCtrl::~OSCPosCtrl() {}

void OSCPosCtrl::oneStep(void* _cmd) {
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumDofs());
    Eigen::VectorXd rf_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rf_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rf_acc_ff = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rf_acc_des = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd rf_pos_act = robot_->getBodyNodeIsometry("r_foot").translation();
    Eigen::VectorXd rf_vel_act = robot_->getBodyNodeSpatialVelocity("r_foot").tail(3); 

    for (int i = 0; i < 3; ++i) {
        rf_pos_des[i] = myUtils::smooth_changing(ini_pos_[i], target_pos_[i],
                                         end_time_, state_machine_time_);
        rf_vel_des[i] = myUtils::smooth_changing_vel(ini_pos_[i], target_pos_[i],
                                                end_time_, state_machine_time_);
        rf_acc_ff[i] = myUtils::smooth_changing_acc(ini_pos_[i], target_pos_[i],
                                                end_time_, state_machine_time_);
        rf_acc_des[i] =
            rf_acc_ff[i] + rf_kp_[i] * (rf_pos_des[i] - rf_pos_act[i]) 
            + rf_kd_[i] * (rf_vel_des[i] - rf_vel_act[i]);
    }

    Eigen::MatrixXd J_rf = robot_->getBodyNodeJacobian("r_foot").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_rf_bar = Eigen::MatrixXd::Zero(robot_->getNumDofs(),3);
    myUtils::weightedInverse(J_rf,robot_->getInvMassMatrix(),J_rf_bar);
    Eigen::MatrixXd J_rf_dot = robot_->getBodyNodeJacobianDot("r_foot").block(3,0,3,robot_->getNumDofs());
    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(robot_->getNumDofs());

    qddot_des = J_rf_bar * (rf_acc_des - J_rf_dot*(robot_->getQdot()));

    gamma = robot_->getMassMatrix() * qddot_des + robot_->getCoriolisGravity();

    ((FixedAtlasCommand*)_cmd)->jtrq = gamma;

    ++ctrl_count_;
    state_machine_time_= ctrl_count_ * AtlasAux::ServoRate;

}

void OSCPosCtrl::firstVisit() {
    state_machine_time_= 0.;
    ctrl_count_ = 0;
    ini_pos_ = robot_->getBodyNodeIsometry("r_foot").translation();
    ini_vel_ = robot_->getBodyNodeSpatialVelocity("r_foot").tail(3); 
}
void OSCPosCtrl::lastVisit() {
}

bool OSCPosCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void OSCPosCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
        myUtils::readParameter(node, "q_kp", q_kp_);
        myUtils::readParameter(node, "q_kd", q_kd_);
        myUtils::readParameter(node, "rf_kp", rf_kp_);
        myUtils::readParameter(node, "rf_kd", rf_kd_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
