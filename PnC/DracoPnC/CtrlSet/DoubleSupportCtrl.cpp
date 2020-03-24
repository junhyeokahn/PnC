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

#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>

DoubleSupportCtrl::DoubleSupportCtrl(RobotSystem* robot,
        WalkingReferenceTrajectoryModule* walking_module) : Controller(robot) {
    myUtils::pretty_constructor(2, "Double Support Ctrl");

    walking_reference_trajectory_module_ = walking_module;
    b_do_plan_ = true;

    tau_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);
    tau_cmd_old_ = Eigen::VectorXd::Zero(Draco::n_adof);
    qddot_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);

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

DoubleSupportCtrl::~DoubleSupportCtrl() {
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

void DoubleSupportCtrl::firstVisit() {
    std::cout << "First Visit of DoubleSupportCtrl" << std::endl;
    ctrl_start_time_ = sp_->curr_time;

    ini_com_pos_ = robot_->getCoMPosition();
    ini_com_vel_ = robot_->getCoMVelocity();
    Eigen::Isometry3d rf_iso =
        robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter);
    Eigen::Isometry3d lf_iso =
        robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter);
    for (int i = 0; i < 3; ++i) {
        goal_com_pos_[i] =
            (rf_iso.translation()[i] + lf_iso.translation()[i]) / 2.0;
    }
    goal_com_pos_[2] = target_com_height_;

    if (sp_->b_walking) {
        b_do_plan_ = true;
    } else {
        b_do_plan_ = false;
    }
}

void DoubleSupportCtrl::lastVisit() {
    std::cout << "Last Visit of DoubleSupportCtrl" << std::endl;
}

bool DoubleSupportCtrl::endOfPhase() {
    if (!sp_->b_walking) {
        return false;
    } else {
        if (sp_->num_residual_steps == sp_->num_total_steps) {
            if (state_machine_time_ > initial_double_support_dur_) {
                return true;
            } else {
                return false;
            }
        } else {
            if (state_machine_time_ > double_support_dur_) {
                return true;
            } else {
                return false;
            }
        }
    }
}

void DoubleSupportCtrl::ctrlInitialization(const YAML::Node& node) {
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

void DoubleSupportCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    if (b_do_plan_) {
        _references_setup();
        b_do_plan_ = false;
    }
    if (sp_->b_walking) {
        _walking_contact_setup();
        _walking_task_setup();
    } else {
        _balancing_contact_setup();
        _balancing_task_setup();
    }
    _compute_torque_ihwbc();

    double alphaTau = (1.0 - myUtils::computeAlphaGivenBreakFrequency(
                100.0, DracoAux::ServoRate));
    tau_cmd_old_ = tau_cmd_*alphaTau + (1.0 - alphaTau)*tau_cmd_old_;

    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = tau_cmd_old_[i];
        ((DracoCommand*)_cmd)->q[i] = sp_->des_jpos[i];
        ((DracoCommand*)_cmd)->qdot[i] = sp_->des_jvel[i];
    }

    _PostProcessing_Command();
}


void DoubleSupportCtrl::_references_setup() {

    // Set references initial condition
    Eigen::Vector3d x_com_start = robot_->getCoMPosition();
    Eigen::Quaternion<double> x_ori_start = myUtils::EulerZYXtoQuat(sp_->q[5],
            sp_->q[4], sp_->q[3]);

    DracoFootstep rfoot_start;
    DracoFootstep lfoot_start;
    Eigen::Vector3d lfoot_pos = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();
    Eigen::Quaterniond lfoot_ori(robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).linear());
    lfoot_start.setPosOriSide(lfoot_pos, lfoot_ori, DRACO_LEFT_FOOTSTEP);

    Eigen::Vector3d rfoot_pos = robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();
    Eigen::Quaterniond rfoot_ori(robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).linear());
    rfoot_start.setPosOriSide(rfoot_pos, rfoot_ori, DRACO_RIGHT_FOOTSTEP);

    // TODO : Change this with initialize
    walking_reference_trajectory_module_->setStartingConfiguration(x_com_start,
            x_ori_start, lfoot_start, rfoot_start);

    // Footstep Sequences
    // TODO : Set this value from API later.
    Eigen::Vector3d foot_translate(0.01, 0.0, 0.0);
    Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );

    double first_ds_dur;
    Eigen::Vector3d first_foot_translate;
    if (sp_->num_residual_steps == sp_->num_total_steps) {
        //first_ds_dur = initial_double_support_dur_;
        first_foot_translate = foot_translate;
        ((DCMWalkingReferenceTrajectoryModule*)walking_reference_trajectory_module_)->
            dcm_reference.t_transfer =
            initial_double_support_dur_ - (2-alpha_ds_)*double_support_dur_;
    } else {
        //first_ds_dur = double_support_dur_;
        first_foot_translate = 2*foot_translate;
        ((DCMWalkingReferenceTrajectoryModule*)walking_reference_trajectory_module_)->
            dcm_reference.t_transfer = (alpha_ds_-1)*double_support_dur_;
    }

    std::vector<DracoFootstep> footstep_list;
    Eigen::Vector3d swing_foot_target_pos;
    Eigen::Quaternion<double> swing_foot_target_quat;
    if (sp_->phase_copy == 2) {
        // Stance: Left
        if (sp_->num_residual_steps == 0) {
            // Add Last Right Step
            swing_foot_target_pos =
                foot_rotate.toRotationMatrix()*(rfoot_start.position) +
                foot_translate;
            swing_foot_target_quat = foot_rotate*rfoot_start.orientation;
            DracoFootstep rfoot_last;
            rfoot_last.setPosOriSide(swing_foot_target_pos,
                    swing_foot_target_quat, DRACO_RIGHT_FOOTSTEP);
            footstep_list.push_back(rfoot_last);
        } else if (sp_->num_residual_steps == 1) {
            // Add First Right Step
            swing_foot_target_pos =
                foot_rotate.toRotationMatrix()*(rfoot_start.position) +
                first_foot_translate;
            swing_foot_target_quat = foot_rotate*rfoot_start.orientation;
            DracoFootstep rfoot_first;
            rfoot_first.setPosOriSide(swing_foot_target_pos,
                    swing_foot_target_quat, DRACO_RIGHT_FOOTSTEP);
            footstep_list.push_back(rfoot_first);
            // Add Last Left Step
            DracoFootstep lfoot_last;
            lfoot_last.setPosOriSide(
                foot_rotate.toRotationMatrix()*(lfoot_start.position) +
                foot_translate, foot_rotate*lfoot_start.orientation,
                DRACO_LEFT_FOOTSTEP);
            footstep_list.push_back(lfoot_last);
        } else {
            // Add First Right Step
            swing_foot_target_pos =
                foot_rotate.toRotationMatrix()*(rfoot_start.position) +
                first_foot_translate;
            swing_foot_target_quat = foot_rotate*rfoot_start.orientation;
            DracoFootstep rfoot_first;
            rfoot_first.setPosOriSide(swing_foot_target_pos,
                    swing_foot_target_quat, DRACO_RIGHT_FOOTSTEP);
            footstep_list.push_back(rfoot_first);
            // Add Second Left Step
            DracoFootstep lfoot_second;
            lfoot_second.setPosOriSide(
                foot_rotate.toRotationMatrix()*(lfoot_start.position) +
                2*foot_translate, foot_rotate*lfoot_start.orientation,
                DRACO_LEFT_FOOTSTEP);
            footstep_list.push_back(lfoot_second);
            // Add Last Right Step
            DracoFootstep rfoot_last;
            rfoot_last.setPosOriSide(
                    foot_rotate.toRotationMatrix()*(rfoot_first.position) +
                    foot_translate, foot_rotate*rfoot_first.orientation,
                    DRACO_RIGHT_FOOTSTEP);
            footstep_list.push_back(rfoot_last);
        }
    } else {
        // Stance: Right
        if (sp_->num_residual_steps == 0) {
            // Add Last Left Step
            swing_foot_target_pos =
                foot_rotate.toRotationMatrix()*(lfoot_start.position) +
                foot_translate;
            swing_foot_target_quat = foot_rotate*lfoot_start.orientation;
            DracoFootstep lfoot_last;
            lfoot_last.setPosOriSide(swing_foot_target_pos,
                    swing_foot_target_quat, DRACO_LEFT_FOOTSTEP);
            footstep_list.push_back(lfoot_last);
        } else if (sp_->num_residual_steps == 1) {
            // Add First Left Step
            swing_foot_target_pos =
                foot_rotate.toRotationMatrix()*(lfoot_start.position) +
                first_foot_translate;
            swing_foot_target_quat = foot_rotate*lfoot_start.orientation;
            DracoFootstep lfoot_first;
            lfoot_first.setPosOriSide(swing_foot_target_pos,
                    swing_foot_target_quat, DRACO_LEFT_FOOTSTEP);
            footstep_list.push_back(lfoot_first);
            // Add Last Right Step
            DracoFootstep rfoot_last;
            rfoot_last.setPosOriSide(
                foot_rotate.toRotationMatrix()*(rfoot_start.position) +
                foot_translate, foot_rotate*rfoot_start.orientation,
                DRACO_RIGHT_FOOTSTEP);
            footstep_list.push_back(rfoot_last);
        } else {
            // Add First Left Step
            swing_foot_target_pos =
                foot_rotate.toRotationMatrix()*(lfoot_start.position) +
                first_foot_translate;
            swing_foot_target_quat = foot_rotate*lfoot_start.orientation;
            DracoFootstep lfoot_first;
            lfoot_first.setPosOriSide(swing_foot_target_pos,
                    swing_foot_target_quat, DRACO_LEFT_FOOTSTEP);
            footstep_list.push_back(lfoot_first);
            // Add Second Right Step
            DracoFootstep rfoot_second;
            rfoot_second.setPosOriSide(
                foot_rotate.toRotationMatrix()*(rfoot_start.position) +
                2*foot_translate, foot_rotate*rfoot_start.orientation,
                DRACO_RIGHT_FOOTSTEP);
            footstep_list.push_back(rfoot_second);
            // Add Last Left Step
            DracoFootstep lfoot_last;
            lfoot_last.setPosOriSide(
                    foot_rotate.toRotationMatrix()*(lfoot_first.position) +
                    foot_translate, foot_rotate*lfoot_first.orientation,
                    DRACO_LEFT_FOOTSTEP);
            footstep_list.push_back(lfoot_last);
        }
    }
    for (int i = 0; i < 3; ++i) {
        sp_->swing_foot_target_pos[i] = swing_foot_target_pos[i];
    }
    sp_->swing_foot_target_quat = swing_foot_target_quat;

    walking_reference_trajectory_module_->setFootsteps(sp_->curr_time, footstep_list);

    for(int i = 0; i < footstep_list.size(); i++){
        printf("Step %i:\n", i);
        footstep_list[i].printInfo();
    }
}

void DoubleSupportCtrl::_walking_contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    for(int i = 0; i < contact_list_.size(); i++){
        ((PointContactSpec*)contact_list_[i])->setMaxFz(max_fz_);
    }
}

void DoubleSupportCtrl::_balancing_contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    for(int i = 0; i < contact_list_.size(); i++){
        ((PointContactSpec*)contact_list_[i])->setMaxFz(max_fz_);
    }
}

void DoubleSupportCtrl::_walking_task_setup() {
    // CoM Task
    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);

    Eigen::Vector3d com_pos_ref, com_vel_ref, com_pos, com_vel;
    walking_reference_trajectory_module_->getMPCRefComPosandVel(
            sp_->curr_time, com_pos_ref, com_vel_ref);
    ((DCMWalkingReferenceTrajectoryModule*)walking_reference_trajectory_module_)->
        dcm_reference.get_ref_dcm(sp_->curr_time, sp_->dcm_des);
    ((DCMWalkingReferenceTrajectoryModule*)walking_reference_trajectory_module_)->
        dcm_reference.get_ref_dcm_vel(sp_->curr_time, sp_->dcm_vel_des);
    ((DCMWalkingReferenceTrajectoryModule*)walking_reference_trajectory_module_)->
        dcm_reference.get_ref_r_vrp(sp_->curr_time, sp_->r_vrp_des);

    com_pos = robot_->getCoMPosition();
    com_vel = robot_->getCoMVelocity();

    for (int i = 0; i < 2; ++i) {
        com_pos_des[i] = com_pos[i];
        com_vel_des[i] = com_vel[i];
    }
    Eigen::VectorXd r_ic = sp_->dcm.head(2);
    Eigen::VectorXd r_id = sp_->dcm_des.head(2);
    Eigen::VectorXd rdot_id = sp_->dcm_vel_des.head(2);
    Eigen::VectorXd r_icp_error = (r_ic - r_id);
    double kp_ic(20.);
    double omega_o = std::sqrt(9.81/target_com_height_);
    Eigen::VectorXd r_CMP_d = r_ic - rdot_id/omega_o + kp_ic * (r_icp_error);
    com_acc_des.head(2) = (9.81/target_com_height_) * (com_pos.head(2) - r_CMP_d);

    com_pos_des[2] = target_com_height_;
    //com_vel_des[2] = com_vel_ref[2];
    com_vel_des[2] = 0.;
    com_acc_des[2] = 0.;

    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // BodyOri Task
    Eigen::VectorXd bodyori_pos_des = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd bodyori_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd bodyori_acc_des = Eigen::VectorXd::Zero(3);

    Eigen::Quaterniond ori_ref;
    Eigen::Vector3d ang_vel_ref, ang_acc_ref;
    walking_reference_trajectory_module_->getMPCRefQuatAngVelAngAcc(sp_->curr_time,
            ori_ref, ang_vel_ref, ang_acc_ref);
    bodyori_pos_des << ori_ref.w(), ori_ref.x(), ori_ref.y(), ori_ref.z();
    for (int i = 0; i < 3; ++i) {
        bodyori_vel_des[i] = ang_vel_ref[i];
        bodyori_acc_des[i] = ang_acc_ref[i];
        //bodyori_acc_des[i] = 0.;
    }
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

    // Update Task List
    task_list_.push_back(com_task_);
    task_list_.push_back(bodyori_task_);
    task_list_.push_back(rfoot_front_task_);
    task_list_.push_back(rfoot_back_task_);
    task_list_.push_back(lfoot_front_task_);
    task_list_.push_back(lfoot_back_task_);

    // Task Weights
    task_weight_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());
    task_weight_heirarchy_[0] = com_task_weight_;
    task_weight_heirarchy_[1] = bodyori_task_weight_;
    task_weight_heirarchy_[2] = rfoot_task_weight_;
    task_weight_heirarchy_[3] = rfoot_task_weight_;
    task_weight_heirarchy_[4] = lfoot_task_weight_;
    task_weight_heirarchy_[5] = lfoot_task_weight_;
}

void DoubleSupportCtrl::_balancing_task_setup(){
    // CoM Task
    double stab_time(4.);
    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);
    if (state_machine_time_ < stab_time) {
        for (int i = 0; i < 3; ++i) {
            com_pos_des[i] = myUtils::smooth_changing(ini_com_pos_[i],
                    goal_com_pos_[i], stab_time, state_machine_time_);
            com_vel_des[i] = myUtils::smooth_changing_vel(ini_com_pos_[i],
                    goal_com_pos_[i], stab_time, state_machine_time_);
            com_acc_des[i] = myUtils::smooth_changing_acc(ini_com_pos_[i],
                    goal_com_pos_[i], stab_time, state_machine_time_);
            sp_->com_pos_des[i] = com_pos_des[i];
            sp_->com_vel_des[i] = com_vel_des[i];
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            com_pos_des[i] = goal_com_pos_[i];
            com_vel_des[i] = 0.;
            com_acc_des[i] = 0.;
            sp_->com_pos_des[i] = com_pos_des[i];
            sp_->com_vel_des[i] = com_vel_des[i];
        }
    }
    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // BodyOri Task
    Eigen::Isometry3d rf_iso =
        robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter);
    Eigen::Isometry3d lf_iso =
        robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter);
    Eigen::Quaternion<double> rf_q = Eigen::Quaternion<double>(rf_iso.linear());
    Eigen::Quaternion<double> lf_q = Eigen::Quaternion<double>(lf_iso.linear());
    Eigen::Quaternion<double> des_q = rf_q.slerp(0.5, lf_q);
    Eigen::VectorXd des_quat_vec = Eigen::VectorXd::Zero(4);
    des_quat_vec << des_q.w(), des_q.x(), des_q.y(), des_q.z();

    Eigen::VectorXd des_so3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd ori_acc_des = Eigen::VectorXd::Zero(3);

    bodyori_task_->updateTask(des_quat_vec, des_so3, ori_acc_des);

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

    // Update Task List
    task_list_.push_back(com_task_);
    task_list_.push_back(bodyori_task_);
    task_list_.push_back(rfoot_front_task_);
    task_list_.push_back(rfoot_back_task_);
    task_list_.push_back(lfoot_front_task_);
    task_list_.push_back(lfoot_back_task_);

    // Task Weights
    task_weight_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());
    task_weight_heirarchy_[0] = com_task_weight_;
    task_weight_heirarchy_[1] = bodyori_task_weight_;
    task_weight_heirarchy_[2] = rfoot_task_weight_;
    task_weight_heirarchy_[3] = rfoot_task_weight_;
    task_weight_heirarchy_[4] = lfoot_task_weight_;
    task_weight_heirarchy_[5] = lfoot_task_weight_;
}

void DoubleSupportCtrl::_compute_torque_ihwbc() {
    // Update Setting
    Eigen::MatrixXd A_rotor = A_;
    for (int i = 0; i < Draco::n_adof; ++i) {
        A_rotor(i + Draco::n_vdof, i + Draco::n_vdof) += sp_->rotor_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();
    ihwbc_->updateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);

    // Enable Torque Limits
    ihwbc_->enableTorqueLimits(true);
    ihwbc_->setTorqueLimits(-100*Eigen::VectorXd::Ones(Draco::n_adof),
            100*Eigen::VectorXd::Ones(Draco::n_adof));

    // Set QP Weights
    ihwbc_->setQPWeights(task_weight_heirarchy_,
            rf_tracking_weight_/robot_->getRobotMass()*9.81);
    ihwbc_->setRegularizationTerms(qddot_reg_weight_, rf_reg_weight_);

    // Solve
    ihwbc_->solve(task_list_, contact_list_, Eigen::VectorXd::Zero(dim_contact_),
            tau_cmd_, qddot_cmd_);
    ihwbc_->getQddotResult(sp_->qddot_cmd);
    ihwbc_->getFrResult(sp_->reaction_forces);

    // Integrate Joint Velocities
    Eigen::VectorXd qdot_des_ref = Eigen::VectorXd::Zero(Draco::n_adof);
    double alphaVelocity = myUtils::computeAlphaGivenBreakFrequency(
            velocity_break_freq_, DracoAux::ServoRate);
    sp_->des_jvel = sp_->des_jvel*alphaVelocity + (1. - alphaVelocity)*qdot_des_ref;
    sp_->des_jvel += (qddot_cmd_ * DracoAux::ServoRate);
    for (int i = 0; i < sp_->des_jvel.size(); ++i) {
        sp_->des_jvel[i] = myUtils::CropValue(sp_->des_jvel[i], -max_jvel_, max_jvel_);
    }

    // Integrate Joint Positions
    Eigen::VectorXd q_des_ref = sp_->q.tail(Draco::n_adof);
    double alphaPosition = myUtils::computeAlphaGivenBreakFrequency(
            position_break_freq_, DracoAux::ServoRate);
    sp_->des_jpos = sp_->des_jpos*alphaPosition + (1.0 - alphaPosition)*q_des_ref;
    sp_->des_jpos += (sp_->des_jvel*DracoAux::ServoRate);
    for (int i = 0; i < sp_->des_jpos.size(); ++i) {
        sp_->des_jpos[i] = myUtils::CropValue(sp_->des_jpos[i],
                q_des_ref[i]-max_jpos_error_, q_des_ref[i]+max_jpos_error_);
    }
}
