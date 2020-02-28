#include <PnC/WBC/OSC.hpp>
#include <PnC/ScorpioPnC/CtrlSet/OSCCtrl.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Utils/IO/DataManager.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <PnC/ScorpioPnC/ScorpioStateProvider.hpp>
#include <PnC/ScorpioPnC/TaskSet/SelectedJointTask.hpp>
#include <PnC/ScorpioPnC/TaskSet/SuctionGripperTask.hpp>

OSCCtrl::OSCCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "OSC POS Ctrl");
    end_time_ = 0;

    target_pos_ = Eigen::VectorXd::Zero(3);
    relative_target_pos_ = Eigen::VectorXd::Zero(3);
    ini_pos_ = Eigen::VectorXd::Zero(3);
    ini_pos_q_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    q_kp_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    q_kd_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    ini_ori_ = Eigen::Quaternion<double> (1,0,0,0);
    target_ori_ = Eigen::Quaternion<double> (1,0,0,0);
    relative_target_ori_ = Eigen::Quaternion<double> (1,0,0,0);

    _build_active_joint_idx();
    ee_pos_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, ScorpioBodyNode::end_effector);
    ee_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3, ScorpioBodyNode::end_effector);
    //ee_ori_task_ = new SuctionGripperTask(robot_);
    joint_task_ = new SelectedJointTask(robot_, active_joint_idx_);

    _build_constraint_matrix();
    osc_ = new OSC(active_joint_, &Jc_);
    sp_ = ScorpioStateProvider::getStateProvider(_robot);
}

OSCCtrl::~OSCCtrl() {}

void OSCCtrl::_build_active_joint_idx(){
    active_joint_idx_.resize(Scorpio::n_adof);
    active_joint_idx_[0] = 0;
    active_joint_idx_[1] = 1;
    active_joint_idx_[2] = 4;
    active_joint_idx_[3] = 5;
    active_joint_idx_[4] = 8;
    active_joint_idx_[5] = 9;
    active_joint_idx_[6] = 10;

    active_joint_.resize(Scorpio::n_dof, true);
    active_joint_[2] = false;
    active_joint_[3] = false;
    active_joint_[6] = false;
    active_joint_[7] = false;
}

void OSCCtrl::_build_constraint_matrix(){
    Jc_ = Eigen::MatrixXd::Zero(6, Scorpio::n_dof);
    Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(6,Scorpio::n_dof);
    Eigen::MatrixXd J_body_1 = robot_->getBodyNodeJacobian("link2").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_1 = robot_->getBodyNodeJacobian("link4_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_1 = J_constriant_1 - J_body_1;
    Jc_.block(0,0,3,robot_->getNumDofs()) =  J_constriant_diff_1;

    Eigen::MatrixXd J_body_2 = robot_->getBodyNodeJacobian("link6").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_2 = robot_->getBodyNodeJacobian("link8_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_2 = J_constriant_2 - J_body_2;
    Jc_.block(3,0,3,robot_->getNumDofs()) =  J_constriant_diff_2;
}

void OSCCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    _task_setup();
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Scorpio::n_adof);
    _compute_torque(gamma);
    _PostProcessing_Command();

    ((ScorpioCommand*)_cmd)->jtrq = gamma;
}

void OSCCtrl::_task_setup(){
    // =========================================================================
    // EE Pos Task
    // =========================================================================
    Eigen::VectorXd ee_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd ee_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd ee_acc_des = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        target_pos_[i] = ini_pos_[i] + relative_target_pos_[i];
        ee_pos_des[i] = myUtils::smooth_changing(ini_pos_[i], target_pos_[i],
                end_time_, state_machine_time_);
        ee_vel_des[i] = myUtils::smooth_changing_vel(ini_pos_[i], target_pos_[i],
                end_time_, state_machine_time_);
        ee_acc_des[i] = myUtils::smooth_changing_acc(ini_pos_[i], target_pos_[i],
                end_time_, state_machine_time_);
    }
    ee_pos_task_->updateTask(ee_pos_des, ee_vel_des, ee_acc_des);

    // =========================================================================
    // EE Ori Task
    // =========================================================================
    double t = myUtils::smooth_changing(0,1,end_time_,state_machine_time_);
    double tdot = myUtils::smooth_changing_vel(0,1,end_time_,state_machine_time_);

    target_ori_ = relative_target_ori_ * ini_ori_ ;
    Eigen::Quaternion<double> quat_ori_error =
        target_ori_ * (ini_ori_.inverse());
    Eigen::VectorXd ang_vel = Eigen::VectorXd::Zero(3);
    ang_vel = dart::math::quatToExp(quat_ori_error);

    Eigen::VectorXd ori_increment = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd curr_ang_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::Quaternion<double> quat_increment;
    Eigen::Quaternion<double> curr_quat_des;
    ori_increment = ang_vel * t;
    quat_increment = dart::math::expToQuat(ori_increment);
    curr_quat_des = quat_increment * ini_ori_;
    curr_ang_vel_des = ang_vel*tdot;

    Eigen::VectorXd quat_des = Eigen::VectorXd::Zero(4);
    quat_des << curr_quat_des.w(), curr_quat_des.x(), curr_quat_des.y(), curr_quat_des.z();
    Eigen::VectorXd ang_acc_des = Eigen::VectorXd::Zero(3);

    ee_ori_task_->updateTask(quat_des, curr_ang_vel_des, ang_acc_des);

    // =========================================================================
    // Joint Task
    // =========================================================================
    Eigen::VectorXd jpos_des = Eigen::VectorXd::Zero(Scorpio::n_adof);
    for (int i = 0; i < Scorpio::n_adof; ++i) {
        jpos_des[i] = ini_pos_q_[active_joint_idx_[i]];
    }
    Eigen::VectorXd jvel_des = Eigen::VectorXd::Zero(Scorpio::n_adof);
    Eigen::VectorXd jacc_des = Eigen::VectorXd::Zero(Scorpio::n_adof);
    joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Stack Task List
    // =========================================================================
    task_list_.push_back(ee_pos_task_);
    task_list_.push_back(ee_ori_task_);
    //task_list_.push_back(joint_task_);
}

void OSCCtrl::_compute_torque(Eigen::VectorXd & gamma){
    _build_constraint_matrix();
    osc_->updateSetting(A_, Ainv_, coriolis_, grav_, &Jc_);
    osc_->makeTorque(task_list_, contact_list_, gamma);
}


void OSCCtrl::firstVisit() {

    state_machine_time_= 0.;
    ctrl_start_time_ = sp_->curr_time;
    ini_pos_ = robot_->getBodyNodeIsometry("end_effector").translation();
    ini_pos_q_ = robot_->getQ();
    ini_ori_ = Eigen::Quaternion<double> (robot_->getBodyNodeIsometry("end_effector").linear());
}

void OSCCtrl::lastVisit() {
}

bool OSCCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        //TEST for box initial pos
        //std::cout << "end eff lin6ar pos" << std::endl;
        //std::cout << robot_->getBodyNodeIsometry("end_effector").translation() << std::endl;
        //exit(0);
        sp_->is_moving = false;
        return true;
    }
    return false;
}

void OSCCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
        myUtils::readParameter(node, "q_kp", q_kp_);
        myUtils::readParameter(node, "q_kd", q_kd_);
        joint_task_->setGain(q_kp_, q_kd_);
        myUtils::readParameter(node, "lin_kp", lin_kp_);
        myUtils::readParameter(node, "lin_kd", lin_kd_);
        ee_pos_task_->setGain(lin_kp_, lin_kd_);
        myUtils::readParameter(node, "ori_kp", ori_kp_);
        myUtils::readParameter(node, "ori_kd", ori_kd_);
        ee_ori_task_->setGain(ori_kp_, ori_kd_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
            << __FILE__ << "]" << std::endl
            << std::endl;
        exit(0);
    }
}
