#include <PnC/ScorpioPnC/OSCPosCtrl.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

OSCPosCtrl::OSCPosCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "OSC POS Ctrl");
    ctrl_count_ = 0;
}

OSCPosCtrl::~OSCPosCtrl() {}

void OSCPosCtrl::oneStep(void* _cmd) {
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumDofs());
    Eigen::VectorXd end_effector_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_acc_ff = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_acc_des = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd end_effector_pos_act = robot_->getBodyNodeIsometry("end_effector").translation();
    Eigen::VectorXd end_effector_vel_act = robot_->getBodyNodeSpatialVelocity("end_effector").tail(3); 

    for (int i = 0; i < 3; ++i) {
            end_effector_pos_des[i] = myUtils::smooth_changing(ini_pos_[i], target_pos_[i],
                                                             end_time_, state_machine_time_);
            end_effector_vel_des[i] = myUtils::smooth_changing_vel(ini_pos_[i], target_pos_[i],
                                                                    end_time_, state_machine_time_);
            end_effector_acc_ff[i] = myUtils::smooth_changing_acc(ini_pos_[i], target_pos_[i],
                                                                    end_time_, state_machine_time_);
            end_effector_acc_des[i] =
                end_effector_acc_ff[i] + end_effector_kp_[i] * (end_effector_pos_des[i] - end_effector_pos_act[i]) 
                + end_effector_kd_[i] * (end_effector_vel_des[i] - end_effector_vel_act[i]);
        }

    Eigen::MatrixXd J_rf = robot_->getBodyNodeJacobian("end_effector").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_rf_bar = Eigen::MatrixXd::Zero(robot_->getNumDofs(),3);
    myUtils::weightedInverse(J_rf,robot_->getInvMassMatrix(),J_rf_bar);
    Eigen::MatrixXd J_rf_dot = robot_->getBodyNodeJacobianDot("end_effector").block(3,0,3,robot_->getNumDofs());
    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(robot_->getNumDofs());
    Eigen::VectorXd qddot_des_rf = Eigen::VectorXd::Zero(robot_->getNumDofs());
    Eigen::VectorXd qddot_des_q = Eigen::VectorXd::Zero(robot_->getNumDofs());
    Eigen::MatrixXd J_q = Eigen::MatrixXd::Identity(robot_->getNumDofs(),robot_->getNumDofs());
    Eigen::MatrixXd N_rf = Eigen::MatrixXd::Identity(robot_->getNumDofs(),robot_->getNumDofs()) - J_rf_bar * J_rf;
    Eigen::MatrixXd J_q_N_rf = J_q * N_rf;
    Eigen::MatrixXd J_q_N_rf_bar = Eigen::MatrixXd::Zero(robot_->getNumDofs(),robot_->getNumDofs());
    myUtils::weightedInverse(J_q_N_rf,robot_->getInvMassMatrix(),J_q_N_rf_bar);

    qddot_des_rf = J_rf_bar * (rf_acc_des - J_rf_dot*(robot_->getQdot()));
    
    for (int i = 0; i < robot_->getNumDofs(); ++i){
            qddot_des_q[i] = q_kp_[i] * (ini_pos_q[i] - robot_->getQ()[i]) + 
                            q_kd_[i] * (ini_vel_q[i] - robot_->getQdot()[i]);
        }
    qddot_des_q = J_q_N_rf * (qddot_des_q);

    qddot_des = qddot_des_rf + qddot_des_q;

    gamma = robot_->getMassMatrix() * qddot_des + robot_->getCoriolisGravity();

    ((ScorpioCommand*)_cmd)->jtrq = gamma;

    ++ctrl_count_;
    state_machine_time_= ctrl_count_ * ScorpioAux::ServoRate;

}

void OSCPosCtrl::firstVisit() {
    state_machine_time_= 0.;
    ctrl_count_ = 0;
    ini_pos_ = robot_->getBodyNodeIsometry("end_effector").translation();
    ini_vel_ = robot_->getBodyNodeSpatialVelocity("end_effector").tail(3); 
    std::cout << "initial end effector pos" << std::endl;
    std::cout << ini_pos_ << std::endl;
    ini_pos_q = robot_->getQ();
    ini_vel_q = robot_->getQdot();
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
            myUtils::readParameter(node, "end_effector_kp", end_effector_kp_);
            myUtils::readParameter(node, "end_effector_kd", end_effector_kd_);
        } catch (std::runtime_error& e) {
                std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                          << __FILE__ << "]" << std::endl
                          << std::endl;
                exit(0);
            }
}
