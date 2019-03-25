#include <PnC/FixedAtlasPnC/CtrlSet/JPosSwingCtrl.hpp>
#include <PnC/FixedAtlasPnC/FixedAtlasInterface.hpp>
#include <PnC/FixedAtlasPnC/FixedAtlasDefinition.hpp>
#include <Utils/IO/DataManager.hpp>

JPosSwingCtrl::JPosSwingCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "JPos Swing Ctrl");

    end_time_ = 1000.;
    ctrl_count_ = 0;

    amp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    freq_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    phase_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

}

JPosSwingCtrl::~JPosSwingCtrl(){}

void JPosSwingCtrl::oneStep(void* _cmd){
    Eigen::VectorXd q_des_vec = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Eigen::VectorXd qdot_des_vec = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    Eigen::VectorXd q = robot_->getQ();
    Eigen::VectorXd qdot = robot_->getQdot();

    double q_des, qdot_des, qddot_ff, omega;
///TODO::amp_,freq_,phase_ definition
///don't need it (already set) in the JointTest::parametersetting
    double ramp_time(4.0);
    for (int i(0); i < robot_->getNumActuatedDofs(); i++){
        omega = 2. * M_PI * freq_[i];

        q_des = set_ini_jpos_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);
        qdot_des = amp_[i] * omega * cos(omega * state_machine_time_ + phase_[i]);
        qddot_ff = -amp_[i] * omega * omega * sin(omega * state_machine_time_ + phase_[i]);

        if (state_machine_time_ < ramp_time) {
            qdot_des = state_machine_time_/ramp_time*qdot_des;
            qddot_ff = state_machine_time_/ramp_time*qddot_ff;
        }

        q_des_vec[i] = q_des;
        qdot_des_vec[i] = qdot_des;
        qddot_des[i] = qddot_ff + kp_[i] * (q_des - q[i]) + kd_[i] * (qdot_des -qdot[i]);
    }

    gamma = robot_->getMassMatrix() * qddot_des + robot_->getCoriolisGravity();

    ((FixedAtlasCommand*)_cmd)->q = q_des_vec;
    ((FixedAtlasCommand*)_cmd)->qdot = qdot_des_vec;
    ((FixedAtlasCommand*)_cmd)->jtrq = gamma;

    ++ctrl_count_;
    state_machine_time_= ctrl_count_ * AtlasAux::ServoRate;

}

void JPosSwingCtrl::firstVisit() {
    state_machine_time_ = 0;
    ctrl_count_ = 0;
    //ini_pos_ = robot_->getQ();
    //std::cout << ini_pos_ << std::endl;
    //ini_vel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
}

void JPosSwingCtrl::lastVisit() {
}

bool JPosSwingCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void JPosSwingCtrl::ctrlInitialization(const YAML::Node& node){
    try {
        myUtils::readParameter(node, "kp", kp_);
        myUtils::readParameter(node, "kd", kd_);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
