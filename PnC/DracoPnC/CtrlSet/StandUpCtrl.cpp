#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/MPC/CMPC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

StandUpCtrl::StandUpCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "Stand Up Ctrl");

    tau_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);
    tau_cmd_old_ = Eigen::VectorXd::Zero(Draco::n_adof);
    qddot_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);
    qdot_des_ = Eigen::VectorXd::Zero(Draco::n_adof);
    q_des_ = Eigen::VectorXd::Zero(Draco::n_adof);

    // Tasks
    com_task_ = new BasicTask(robot_, BasicTaskType::COM, 3);
    bodyori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3, DracoBodyNode::Torso);
    rfoot_front_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::rFootFront);
    rfoot_back_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::rFootBack);
    lfoot_front_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::lFootFront);
    lfoot_back_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::lFootBack);

    // Contacts
    rfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootFront, 0.7);
    rfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootBack, 0.7);
    lfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootFront, 0.7);
    lfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootBack, 0.7);

    dim_contact_ = rfoot_front_contact_->getDim() + lfoot_front_contact_->getDim() +
                   rfoot_back_contact_->getDim() + lfoot_back_contact_->getDim();

    // IHWBC
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for (int i(0); i < robot_->getNumVirtualDofs(); ++i) act_list[i] = false;
    ihwbc_ = new IHWBC(act_list);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

StandUpCtrl::~StandUpCtrl() {
    // Tasks
    delete com_task_;
    delete bodyori_task_;
    delete rfoot_front_task_;
    delete rfoot_back_task_;
    delete lfoot_front_task_;
    delete lfoot_back_task_;

    // Contacts
    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;

    // IHWBC
    delete ihwbc_;
}

void StandUpCtrl::firstVisit() {
    std::cout << "First Visit of StandUpCtrl" << std::endl;
    ctrl_start_time_ = sp_->curr_time;
    ini_com_pos_ = robot_->getCoMPosition();

    Eigen::VectorXd rankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rAnkle).translation();
    Eigen::VectorXd lankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lAnkle).translation();
    for (int i = 0; i < 2; ++i) {
        target_com_pos_[i] = (rankle_pos[i] + lankle_pos[i]) / 2.;
    }
    target_com_pos_[2] = target_com_height_;
}

void StandUpCtrl::lastVisit() {
    std::cout << "Last Visit of StandUpCtrl" << std::endl;
}

bool StandUpCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void StandUpCtrl::ctrlInitialization(const YAML::Node& node) {
    // Maximum Reaction Force
    // TODO : Is this right number?
    myUtils::readParameter(node, "max_fz", max_fz_);

    // Task Weights
    myUtils::readParameter(node, "com_task_weight", com_task_weight_);
    myUtils::readParameter(node, "bodyori_task_weight", bodyori_task_weight_);
    myUtils::readParameter(node, "rfoot_task_weight", rfoot_task_weight_);
    myUtils::readParameter(node, "lfoot_task_weight", lfoot_task_weight_);
    myUtils::readParameter(node, "rf_tracking_weight", rf_tracking_weight_);
    myUtils::readParameter(node, "qddot_reg_weight", qddot_reg_weight_);
    myUtils::readParameter(node, "rf_reg_weight", rf_reg_weight_);

    // Task Gains
    Eigen::VectorXd kp, kd;

    myUtils::readParameter(node, "com_task_kp_gain", kp);
    myUtils::readParameter(node, "com_task_kd_gain", kd);
    com_task_->setGain(kp, kd);

    myUtils::readParameter(node, "bodyori_task_kp_gain", kp);
    myUtils::readParameter(node, "bodyori_task_kd_gain", kd);
    bodyori_task_->setGain(kp, kd);

    myUtils::readParameter(node, "rfoot_task_kp_gain", kp);
    myUtils::readParameter(node, "rfoot_task_kd_gain", kd);
    rfoot_front_task_->setGain(kp, kd);
    rfoot_back_task_->setGain(kp, kd);

    myUtils::readParameter(node, "lfoot_task_kp_gain", kp);
    myUtils::readParameter(node, "lfoot_task_kd_gain", kd);
    lfoot_front_task_->setGain(kp, kd);
    lfoot_back_task_->setGain(kp, kd);

    // Safety Parameters on Admittance Control
    myUtils::readParameter(node, "velocity_break_freq", velocity_break_freq_);
    myUtils::readParameter(node, "position_break_freq", position_break_freq_);
    myUtils::readParameter(node, "max_jvel", max_jvel_);
    myUtils::readParameter(node, "max_jpos_error", max_jpos_error_);
}

void StandUpCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    // TODO: Should I split this into two? (e.g. contact transition, com moving)
    _contact_setup();
    _task_setup();
    _compute_torque_ihwbc();

    // TODO: Is this right?
    double alphaTau = (1.0 - myUtils::computeAlphaGivenBreakFrequency(
                100.0, DracoAux::ServoRate));
    tau_cmd_old_ = tau_cmd_*alphaTau + (1.0 - alphaTau)*tau_cmd_old_;

    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = tau_cmd_old_[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void StandUpCtrl::_contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    double smooth_max_fz = myUtils::smooth_changing(0.0, max_fz_,
            end_time_, state_machine_time_);

    for(int i = 0; i < contact_list_.size(); i++){
        ((PointContactSpec*)contact_list_[i])->setMaxFz(smooth_max_fz);
    }
}

void StandUpCtrl::_task_setup() {
    // CoM Task
    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        com_pos_des[i] = myUtils::smooth_changing(ini_com_pos[i],
                target_com_pos_[i], end_time_, state_machine_time_);
        com_vel_des[i] = myUtils::smooth_changing_vel(ini_com_pos[i],
                target_com_pos_[i], end_time_, state_machine_time_);
        com_acc_des[i] = myUtils::smooth_changing_acc(ini_com_pos[i],
                target_com_pos_[i], end_time_, state_machine_time_);
        sp_->com_pos_des[i] = com_pos_des[i];
        sp_->com_vel_des[i] = com_vel_des[i];
    }
    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // BodyOri Task
    Eigen::VectorXd bodyori_pos_des = Eigen::VectorXd::Zero(4);
    bodyori_pos_des << 1, 0, 0, 0;
    Eigen::VectorXd bodyori_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd bodyori_acc_des = Eigen::VectorXd::Zero(3);
    bodyori_task_->updateTask(
            bodyori_pos_des, bodyori_vel_des, bodyori_acc_des);

    // Foot Task
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rfoot_front_pos =
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootFront).translation();
    Eigen::VectorXd rfoot_back_pos =
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootBack).translation();
    Eigen::VectorXd lfoot_front_pos =
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootFront).translation();
    Eigen::VectorXd lfoot_back_pos =
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootBack).translation();
    rfoot_front_task_->updateTask(rfoot_front_pos, zero_vec, zero_vec);
    rfoot_back_task_->updateTask(rfoot_back_pos, zero_vec, zero_vec);
    lfoot_front_task_->updateTask(lfoot_front_pos, zero_vec, zero_vec);
    lfoot_back_task_->updateTask(lfoot_back_pos, zero_vec, zero_vec);

    // Task Weights
    // TODO : Should I use myUtils::smooth_changing?
    task_weight_heirarchy_[0] = com_task_weight_;
    task_weight_heirarchy_[1] = bodyori_task_weight_;
    task_weight_heirarchy_[2] = rfoot_task_weight_;
    task_weight_heirarchy_[3] = rfoot_task_weight_;
    task_weight_heirarchy_[4] = lfoot_task_weight_;
    task_weight_heirarchy_[5] = lfoot_task_weight_;

    // Update Task List
    task_list_.push_back(com_task_);
    task_list_.push_back(bodyori_task_);
    task_list_.push_back(rfoot_front_task_);
    task_list_.push_back(rfoot_back_task_);
    task_list_.push_back(lfoot_front_task_);
    task_list_.push_back(lfoot_back_task_);
}

void StandUpCtrl::_compute_torque_ihwbc() {
    // Update Setting
    Eigen::MatrixXd A_rotor = A_;
    for (int i = 0; i < Draco::n_adof; ++i) {
        A_rotor(i + Draco::n_vdof, i + Draco::n_vdof) += sp_->robot_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();
    ihwbc->updateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);

    // Enable Torque Limits
    ihwbc->enableTorqueLimits(true);
    ihwbc->setTorqueLimits(-100*Eigen::VectorXd::Ones(Draco::n_adof),
            100*Eigen::VectorXd::Ones(Draco::n_adof));

    // Set QP Weights
    ihwbc->setQPWeights(task_weight_heirarchy_,
            rf_tracking_weight_/robot_->getRobotMass()*9.81);
    ihwbc->setRegularizationTerms(qddot_reg_weight_, rf_reg_weight_);

    // Solve
    // TODO: What would be reaction force desired?
    ihwbc->solve(task_list_, contact_list_, Eigen::VectorXd::Zero(dim_contact_),
            tau_cmd_, qddot_cmd_);
    ihwbc->getQddotResult(sp_->qddot_cmd);
    ihwbc->getFrResult(sp_->reaction_forces);

    // Integrate Joint Velocities
    Eigen::VectorXd qdot_des_ref = Eigen::VectorXd::Zero(Draco::n_adof);
    double alphaVelocity = myUtils::computeAlphaGivenBreakFrequency(
            velocity_break_freq_, DracoAux::ServoRate);
    // TODO: What would be des_jvel_ set initially?
    des_jvel_ = des_jvel_*alphaVelocity + (1. - alphaVelocity)*qdot_des_ref;
    des_jvel_ += (qddot_cmd_ * DracoAux::ServoRate);
    for (int i = 0; i < des_jvel_.size(); ++i) {
        des_jvel[i] = myUtils::CropValue(des_jvel_[i], -max_jvel, max_jvel);
    }

    // Integrate Joint Positions
    Eigen::VectorXd q_des_ref = sp_->q.tail(Draco::n_adof);
    double alphaPosition = myUtils::computeAlphaGivenBreakFrequency(
            position_break_freq_, DracoAux::ServoRate);
    // TODO: What would be des_jvel_ set initially?
    des_jpos_ = des_jpos_*alphaPosition + (1.0 - alphaPosition)*q_des_ref;
    des_jpos_ += (des_jvel_*DracoAux::ServoRate);
    for (int i = 0; i < des_jpos_.size(); ++i) {
        des_jpos_[i] = myUtils::CropValue(des_jpos_[i],
                q_des_ref[i]-max_jpos_error_, q_des_ref[i]+max_jpos_error_);
    }
}
