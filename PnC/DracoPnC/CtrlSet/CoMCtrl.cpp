#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

CoMCtrl::CoMCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "COM Ctrl");

    stab_dur_ = 5.;
    end_time_ = 1000.;
    ctrl_start_time_ = 0.;

    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    ini_com_pos_ = Eigen::VectorXd::Zero(3);
    ini_com_vel_ = Eigen::VectorXd::Zero(3);
    goal_com_pos_ = Eigen::VectorXd::Zero(3);

    // task
    com_task_ = new BasicTask(robot_, BasicTaskType::COM, 3);
    total_joint_task_ = new BasicTask(robot_, BasicTaskType::JOINT, Draco::n_adof);

    // contact
    rfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootFront, 0.7);
    rfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootBack, 0.7);
    lfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootFront, 0.7);
    lfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootBack, 0.7);

    contact_list_.clear();
    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    dim_contact_ = rfoot_front_contact_->getDim() + lfoot_front_contact_->getDim() +
                   rfoot_back_contact_->getDim() + lfoot_back_contact_->getDim();

    // wbc
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for (int i(0); i < robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);

    wblc_data_ = new WBLC_ExtraData();

    wblc_data_->W_qddot_ =
        Eigen::VectorXd::Constant(robot_->getNumDofs(), 100.0);

    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_rf_[rfoot_front_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getFzIndex()] =
        0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
        lfoot_front_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
        lfoot_front_contact_->getDim() + lfoot_back_contact_->getFzIndex()] = 0.01;

    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);

    wblc_data_->tau_min_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

CoMCtrl::~CoMCtrl() {
    delete com_task_;
    delete total_joint_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
}

void CoMCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    contact_setup();
    task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void CoMCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    Eigen::MatrixXd A_rotor = A_;
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        A_rotor(i + robot_->getNumVirtualDofs(),
                i + robot_->getNumVirtualDofs()) += sp_->rotor_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();

    wblc_->updateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(robot_->getNumVirtualDofs(),
                                        robot_->getNumActuatedDofs())) +
        Kd_.cwiseProduct(des_jvel_ -
                         sp_->qdot.tail(robot_->getNumActuatedDofs()));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);

    // myUtils::pretty_print(des_jacc_, std::cout, "des_jacc");
    // myUtils::pretty_print(des_jacc_cmd, std::cout, "des_jacc_cmd");

    sp_->qddot_cmd = wblc_data_->qddot_;
    //for (int i = 0; i < wblc_data_->Fr_.size(); ++i) {
        //sp_->reaction_forces[i] = wblc_data_->Fr_[i];
    //}
}

void CoMCtrl::task_setup() {

    // =========================================================================
    // Com Task
    // =========================================================================
    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);

    if (state_machine_time_ < stab_dur_) {
        for (int i = 0; i < 3; ++i) {
            com_pos_des[i] = myUtils::smooth_changing(ini_com_pos_[i], goal_com_pos_[i], stab_dur_, state_machine_time_);
            com_vel_des[i] = myUtils::smooth_changing_vel(ini_com_vel_[i], 0., stab_dur_, state_machine_time_);
            com_acc_des[i] = 0.;
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            com_pos_des[i] = goal_com_pos_[i];
            com_vel_des[i] = 0.;
            com_acc_des[i] = 0.;
        }
    }
    for (int i = 0; i < 3; ++i) {
    sp_->com_pos_des[i] = com_pos_des[i];
    sp_->com_vel_des[i] = com_vel_des[i];
    }

    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = jpos_ini_;
    Eigen::VectorXd jvel_des(Draco::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(com_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);
}

void CoMCtrl::contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);
}

void CoMCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    ctrl_start_time_ = sp_->curr_time;

    Eigen::Vector3d com_pos = robot_->getCoMPosition();
    Eigen::Vector3d com_vel = robot_->getCoMVelocity();
    //Eigen::VectorXd rankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
    //Eigen::VectorXd lankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
    Eigen::VectorXd rankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rAnkle).translation();
    Eigen::VectorXd lankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lAnkle).translation();
    for (int i = 0; i < 3; ++i) {
        ini_com_pos_[i] = com_pos[i];
        ini_com_vel_[i] = com_vel[i];
        goal_com_pos_[i] = (rankle_pos[i] + lankle_pos[i]) / 2.0;
    }
    //myUtils::pretty_print(ini_com_pos_, std::cout, "ini_com_pos");
    //exit(0);
    goal_com_pos_[0] += des_com_offset_x_;
    goal_com_pos_[2] = target_com_height_;
}

void CoMCtrl::lastVisit() {}

bool CoMCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void CoMCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());

    Eigen::VectorXd com_kp = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_kd = Eigen::VectorXd::Zero(3);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "com_kp", com_kp);
        myUtils::readParameter(node, "com_kd", com_kd);
        com_task_->setGain(com_kp, com_kd);
        myUtils::readParameter(node, "desired_offset_x", des_com_offset_x_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
