#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>

BodyCtrl::BodyCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "Body Ctrl");

    end_time_ = 1000.;
    ctrl_start_time_ = 0.;
    b_set_height_target_ = false;
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // task
    body_rpz_task_ = new BodyRPZTask(robot);
    selected_jidx_.resize(2);
    selected_jidx_[0] = DracoDoF::rHipYaw;
    selected_jidx_[1] = DracoDoF::lHipYaw;
    selected_joint_task_ = new SelectedJointTask(robot, selected_jidx_);

    // contactk
    rfoot_contact_ =
        new PointContactSpec(robot_, DracoBodyNode::rFootCenter, 0.3);
    lfoot_contact_ =
        new PointContactSpec(robot_, DracoBodyNode::lFootCenter, 0.3);

    contact_list_.clear();
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    fz_idx_in_cost_.clear();
    dim_contact_ = 0;
    for (int i = 0; i < contact_list_.size(); ++i) {
        fz_idx_in_cost_.push_back(dim_contact_ +
                                  contact_list_[i]->getFzIndex());
        dim_contact_ += contact_list_[i]->getDim();
    }

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
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    for (int i = 0; i < contact_list_.size(); ++i) {
        wblc_data_->W_rf_[fz_idx_in_cost_[i]] = 0.01;
    }
    wblc_data_->tau_min_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

BodyCtrl::~BodyCtrl() {
    delete body_rpz_task_;
    delete selected_joint_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BodyCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    _double_contact_setup();
    _body_task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
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

    sp_->qddot_cmd = wblc_data_->qddot_;
    for (int i = 0; i < wblc_data_->Fr_.size(); ++i) {
        sp_->reaction_forces[i] = wblc_data_->Fr_[i];
    }
}

void BodyCtrl::_body_task_setup() {
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    Eigen::VectorXd yaw_pos = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd yaw_vel = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd yaw_acc = Eigen::VectorXd::Zero(2);

    selected_joint_task_->updateTask(yaw_pos, yaw_vel, yaw_acc);

    // Calculate IK for a desired height and orientation.
    double body_height_cmd;

    // Set Desired Orientation
    Eigen::Quaternion<double> des_quat(1, 0, 0, 0);

    Eigen::VectorXd pos_des(7);
    pos_des.setZero();
    Eigen::VectorXd vel_des(6);
    vel_des.setZero();
    Eigen::VectorXd acc_des(6);
    acc_des.setZero();

    if (b_set_height_target_)
        body_height_cmd = target_body_height_;
    else
        body_height_cmd = ini_body_height_;

    // Orientation
    pos_des[0] = des_quat.w();
    pos_des[1] = des_quat.x();
    pos_des[2] = des_quat.y();
    pos_des[3] = des_quat.z();

    // Position
    pos_des[4] = 0.;
    pos_des[5] = ini_body_pos_[1];
    pos_des[6] = body_height_cmd;

    double amp(0.0);
    double omega(0.5 * 2. * M_PI);

    pos_des[6] += amp * sin(omega * state_machine_time_);
    vel_des[5] = amp * omega * cos(omega * state_machine_time_);

    body_rpz_task_->updateTask(pos_des, vel_des, acc_des);

    // task push back
    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(body_rpz_task_);

    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);

    // myUtils::pretty_print(jpos_ini_, std::cout, "jpos ini");
    // myUtils::pretty_print(des_jpos_, std::cout, "des jpos");
    // myUtils::pretty_print(des_jvel_, std::cout, "des jvel");
    // myUtils::pretty_print(des_jacc_, std::cout, "des jacc");
    //
    // Eigen::Vector3d com_pos;
    // robot_->getCoMPosition(com_pos);
    // dynacore::pretty_print(com_pos, std::cout, "com_pos");
}

void BodyCtrl::_double_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    ctrl_start_time_ = sp_->curr_time;

    ini_body_pos_ = robot_->getQ().head(3);
}

void BodyCtrl::lastVisit() {
    // std::cout << "CoM Position @ Last visit in Body Ctrl" << std::endl;
    // std::cout << robot_->getCoMPosition() << std::endl;
    // exit(0);
}

bool BodyCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}
void BodyCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
