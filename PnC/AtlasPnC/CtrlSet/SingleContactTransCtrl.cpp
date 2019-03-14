#include <Configuration.h>
#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/IOUtilities.hpp>

SingleContactTransCtrl::SingleContactTransCtrl(RobotSystem* robot,
                                               int moving_foot, bool b_increase)
    : Controller(robot),
      b_set_height_target_(false),
      moving_foot_(moving_foot),
      b_increase_(b_increase),
      end_time_(100.),
      ctrl_start_time_(0.),
      des_jpos_(Atlas::n_adof),
      des_jvel_(Atlas::n_adof),
      des_jacc_(Atlas::n_adof),
      Kp_(Atlas::n_adof),
      Kd_(Atlas::n_adof) {
    myUtils::pretty_constructor(2, "Single Contact Trans Ctrl");
    total_joint_task_ =
        new BasicTask(robot_, BasicTaskType::JOINT, Atlas::n_adof);
    body_pos_task_ =
        new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, AtlasBodyNode::pelvis);
    body_ori_task_ =
        new BasicTask(robot_, BasicTaskType::LINKORI, 3, AtlasBodyNode::pelvis);
    torso_ori_task_ =
        new BasicTask(robot_, BasicTaskType::LINKORI, 3, AtlasBodyNode::utorso);

    rfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::r_sole,
                                            0.125, 0.075, 0.9);
    lfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::l_sole,
                                            0.125, 0.075, 0.9);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(Atlas::n_dof, true);
    for (int i(0); i < Atlas::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Atlas::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Atlas::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Atlas::n_adof, 2500.);

    sp_ = AtlasStateProvider::getStateProvider(robot_);
    // printf("[Transition Controller] Constructed\n");
}

SingleContactTransCtrl::~SingleContactTransCtrl() {
    delete total_joint_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;
}

void SingleContactTransCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma;

    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < Atlas::n_adof; ++i) {
        ((AtlasCommand*)_cmd)->jtrq[i] = gamma[i];
        ((AtlasCommand*)_cmd)->q[i] = des_jpos_[i];
        ((AtlasCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void SingleContactTransCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Atlas::n_vdof, Atlas::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Atlas::n_adof));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);

    // dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    // dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    // dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    // dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    // dynacore::pretty_print(gamma, std::cout, "gamma");
}

void SingleContactTransCtrl::_task_setup() {
    des_jpos_ = sp_->jpos_ini;
    des_jvel_.setZero();
    des_jacc_.setZero();

    // Calculate IK for a desired height and orientation.
    double body_height_cmd;
    if (b_set_height_target_)
        body_height_cmd = target_body_height_;
    else
        body_height_cmd = ini_body_pos_[2];

    Eigen::VectorXd vel_des(3);
    vel_des.setZero();
    Eigen::VectorXd acc_des(3);
    acc_des.setZero();
    Eigen::VectorXd des_pos = ini_body_pos_;
    des_pos[2] = body_height_cmd;
    body_pos_task_->updateTask(des_pos, vel_des, acc_des);

    // Set Desired Orientation
    Eigen::VectorXd des_quat(4);
    des_quat << 1., 0., 0., 0;

    Eigen::VectorXd ang_vel_des(body_ori_task_->getDim());
    ang_vel_des.setZero();
    Eigen::VectorXd ang_acc_des(body_ori_task_->getDim());
    ang_acc_des.setZero();
    body_ori_task_->updateTask(des_quat, ang_vel_des, ang_acc_des);
    torso_ori_task_->updateTask(des_quat, ang_vel_des, ang_acc_des);

    // Joint
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Atlas::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Atlas::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // Task List Update
    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(total_joint_task_);

    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);

    if (b_increase_) {
        if (moving_foot_ == AtlasBodyNode::r_sole) {
            int swing_jidx = AtlasDoF::r_leg_hpz - Atlas::n_vdof;
            double h(state_machine_time_ / end_time_);

            for (int i(0); i < Atlas::num_leg_joint; ++i) {
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] +=
                    (1. - h) * sp_->des_jpos_prev[swing_jidx + i];
            }
        } else if (moving_foot_ == AtlasBodyNode::l_sole) {
            int swing_jidx = AtlasDoF::l_leg_hpz - Atlas::n_vdof;
            double h(state_machine_time_ / end_time_);

            for (int i(0); i < Atlas::num_leg_joint; ++i) {
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] +=
                    (1. - h) * sp_->des_jpos_prev[swing_jidx + i];
            }
        }
    }

    // dynacore::pretty_print(jpos_ini, std::cout, "jpos ini");
    // dynacore::pretty_print(des_jpos_, std::cout, "des jpos");
    // dynacore::pretty_print(des_jvel_, std::cout, "des jvel");
    // dynacore::pretty_print(des_jacc_, std::cout, "des jacc");
}

void SingleContactTransCtrl::_contact_setup() {
    double alpha = 0.5 * (1 - cos(M_PI * state_machine_time_ / end_time_));
    double upper_lim(100.);
    double rf_weight(100.);
    double rf_weight_z(100.);
    double foot_weight(1000.);

    if (b_increase_) {
        upper_lim = min_rf_z_ + alpha * (max_rf_z_ - min_rf_z_);
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;
        foot_weight = 0.001 * (1. - alpha) + 1000. * alpha;
    } else {
        upper_lim = max_rf_z_ - alpha * (max_rf_z_ - min_rf_z_);
        rf_weight = (alpha)*5. + (1. - alpha) * 1.0;
        rf_weight_z = (alpha)*0.5 + (1. - alpha) * 0.01;
        foot_weight = 0.001 * (alpha) + 1000. * (1. - alpha);
    }
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    int jidx_offset(0);
    if (moving_foot_ == AtlasBodyNode::l_sole) {
        jidx_offset = rfoot_contact_->getDim();
        for (int i(0); i < lfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] =
            rf_weight_z;

        ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(upper_lim);
    } else if (moving_foot_ == AtlasBodyNode::r_sole) {
        for (int i(0); i < rfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] =
            rf_weight_z;

        ((SurfaceContactSpec*)rfoot_contact_)->setMaxFz(upper_lim);
    }
}

void SingleContactTransCtrl::firstVisit() {
    ctrl_start_time_ = sp_->curr_time;
    ini_body_pos_ =
        robot_->getBodyNodeIsometry(AtlasBodyNode::pelvis).translation();
}

void SingleContactTransCtrl::lastVisit() { sp_->des_jpos_prev = des_jpos_; }

bool SingleContactTransCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void SingleContactTransCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "max_rf_z", max_rf_z_);
        myUtils::readParameter(node, "min_rf_z", min_rf_z_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
