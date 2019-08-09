#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

BalanceCtrl::BalanceCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "Balance Ctrl");
    ctrl_start_time_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Atlas::n_adof);


    target_pos_ = Eigen::VectorXd::Zero(3);
    com_pos_des_ = Eigen::VectorXd::Zero(3);
    com_vel_des_ = Eigen::VectorXd::Zero(3);
    com_acc_des_ = Eigen::VectorXd::Zero(3);
    des_jacc_cmd_ = Eigen::VectorXd::Zero(Atlas::n_adof);

    // TASK
    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Atlas::n_adof);
    com_task_ =
        new CoMTask(robot);
    // new BasicTask(robot, BasicTaskType::COM, 3, AtlasBodyNode::pelvis);

    std::vector<bool> act_list;
    act_list.resize(Atlas::n_dof, true);
    for (int i(0); i < Atlas::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    rfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::r_sole,
                                            0.125, 0.075, 0.9);
    lfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::l_sole,
                                            0.125, 0.075, 0.9);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Atlas::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Atlas::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Atlas::n_adof, 2500.);

    sp_ = AtlasStateProvider::getStateProvider(robot);
    DataManager* dm = DataManager::GetDataManager();
    dm->RegisterData(&com_pos_des_, VECT, "com_pos_des",3);
    dm->RegisterData(&com_vel_des_, VECT, "com_vel_des",3);
    dm->RegisterData(&com_acc_des_, VECT, "com_acc_des",3);

    dm->RegisterData(&des_jacc_cmd_, VECT, "jacc_des",Atlas::n_adof);
}

BalanceCtrl::~BalanceCtrl() {
    delete total_joint_task_;
    delete com_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BalanceCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Atlas::n_adof);
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

void BalanceCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    //Eigen::VectorXd des_jacc_cmd =
    des_jacc_cmd_ = 
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Atlas::n_vdof, Atlas::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Atlas::n_adof));

    wblc_->makeWBLC_Torque(des_jacc_cmd_, contact_list_, gamma, wblc_data_);
}

void BalanceCtrl::_task_setup() {

    // =========================================================================
    // CoM Task
    // =========================================================================

    if (state_machine_time_ < target_time_){
        _GetBsplineTrajectory();
        com_task_->updateTask(com_pos_des_, com_vel_des_, com_acc_des_);
    }
    else{
        com_pos_des_[2] = target_com_height_+amplitude_ * sin(omega_*(state_machine_time_ - target_time_));
        com_vel_des_[2] = amplitude_ * omega_ *  cos(omega_*(state_machine_time_- target_time_));
        com_acc_des_[2] = -amplitude_ * omega_ * omega_ *  sin(omega_*(state_machine_time_- target_time_));
        //myUtils::pretty_print(pos_des, std::cout, "des_pos");
        //myUtils::pretty_print(vel_des, std::cout, "des_vel");
        //myUtils::pretty_print(acc_des, std::cout, "des_acc");

        com_task_->updateTask(com_pos_des_, com_vel_des_, com_acc_des_);
    }
    

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Atlas::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Atlas::n_adof);
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

void BalanceCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BalanceCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Atlas::n_vdof, Atlas::n_adof);
    ctrl_start_time_ = sp_->curr_time;

    Eigen::Vector3d com_pos = robot_->getCoMPosition();
    ini_com_pos_ = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        ini_com_pos_[i] =  com_pos[i];
    }
    target_pos_ = ini_com_pos_;
    target_pos_[2] = target_com_height_;
    _SetBspline(ini_com_pos_,target_pos_);
    omega_ = 2 * M_PI * frequency_;
}

void BalanceCtrl::lastVisit() {}

bool BalanceCtrl::endOfPhase() {
     //if (state_machine_time_ > end_time_) {
     //return true;
    //}
    return false;
}

void BalanceCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Atlas::n_vdof, Atlas::n_adof);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "amplitude", amplitude_);
        myUtils::readParameter(node, "frequency", frequency_);
        myUtils::readParameter(node, "target_time", target_time_);
        myUtils::readParameter(node, "target_com_height", target_com_height_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void BalanceCtrl::_SetBspline(const Eigen::VectorXd st_pos,
                              const Eigen::VectorXd des_pos) {
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    Eigen::Vector3d middle_pos;

    middle_pos = (st_pos + des_pos) / 2.;

    // Initial and final position & velocity & acceleration
    for (int i(0); i < 3; ++i) {
        // Initial
        init[i] = st_pos[i];
        init[i + 3] = 0.;
        init[i + 6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i + 3] = 0.;
        fin[i + 6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    fin[5] = amplitude_ * omega_;
    com_traj_.SetParam(init, fin, middle_pt, target_time_);

    delete[] * middle_pt;
    delete[] middle_pt;
}

void BalanceCtrl::_GetBsplineTrajectory(){
    double pos[3];
    double vel[3];
    double acc[3];

    com_traj_.getCurvePoint(state_machine_time_, pos);
    com_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    com_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for (int i(0); i < 3; ++i) {
        com_pos_des_[i] = pos[i];
        com_vel_des_[i] = vel[i];
        com_acc_des_[i] = acc[i];
    }
}
