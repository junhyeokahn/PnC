#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

BalanceCtrl::BalanceCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "Balance Ctrl");
    ctrl_start_time_ = 0.;
    end_time_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(Draco::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Draco::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Draco::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Draco::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Draco::n_adof);

    ini_com_pos_ = Eigen::VectorXd::Zero(3);
    des_com_pos_ = Eigen::VectorXd::Zero(3);
    des_com_vel_ = Eigen::VectorXd::Zero(3);
    des_com_acc_ = Eigen::VectorXd::Zero(3);

    com_pos_dev_ = Eigen::VectorXd::Zero(3);

    //ini_torso_quat = Eigen::Quaternion<double> (1,0,0,0);

    // TASK
    com_task_ =
        new BasicTask(robot, BasicTaskType::COM, 3);
    //torso_ori_task_ =
        //new BasicTask(robot,BasicTaskType::LINKORI, 3, DracoBodyNode::Torso);
    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);

    std::vector<bool> act_list;
    act_list.resize(Draco::n_dof, true);
    for (int i(0); i < Draco::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    //Surface Contact
    //rfoot_contact_ = new SurfaceContactSpec(
        //robot_, DracoBodyNode::rFootCenter, 0.085, 0.02, 0.7);
    //lfoot_contact_ = new SurfaceContactSpec(
        //robot_, DracoBodyNode::lFootCenter, 0.085, 0.02, 0.7);

    //Line Contact
    rfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootFront, 0.7);    
    rfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootBack, 0.7);    
    lfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootFront, 0.7);    
    lfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootBack, 0.7);    

    //dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();
    dim_contact_ = rfoot_front_contact_->getDim() + lfoot_front_contact_->getDim() +
                    rfoot_back_contact_->getDim() + lfoot_back_contact_->getDim();

    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Draco::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_front_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getFzIndex()] =
        0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
                lfoot_front_contact_->getFzIndex()] = 0.01;

    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
                lfoot_front_contact_->getDim() + lfoot_back_contact_->getFzIndex()] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Draco::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Draco::n_adof, 2500.);

    sp_ = DracoStateProvider::getStateProvider(robot);

}

BalanceCtrl::~BalanceCtrl() {
    delete total_joint_task_;
    delete com_task_;
    //delete torso_ori_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
    //delete rfoot_contact_;
    //delete lfoot_contact_;
}

void BalanceCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Draco::n_adof);
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma); 
    for (int i(0); i < Draco::n_adof; ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void BalanceCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Draco::n_vdof, Draco::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Draco::n_adof));

    // myUtils::pretty_print(des_jacc_cmd, std::cout, "balance");
    // exit(0);
    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);
}

void BalanceCtrl::_task_setup() {
    // =========================================================================
    // CoMTask
    // =========================================================================
        //ori_traj
        //xyz_traj
    if(state_machine_time_ < end_time_){
    _GetBsplineTrajectory();
    // for com plotting
    for (int i = 0; i < 3; ++i) {
        (sp_->com_pos_des)[i] = des_com_pos_[i];
        (sp_->com_vel_des)[i] = des_com_vel_[i];}
    } else{
    des_com_pos_ = ini_com_pos_ + com_pos_dev_;
    des_com_vel_ = Eigen::VectorXd::Zero(3);
    des_com_acc_ = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        (sp_->com_pos_des)[i] = des_com_pos_[i];
        (sp_->com_vel_des)[i] = des_com_vel_[i];}
    }

    com_task_->updateTask(des_com_pos_,des_com_vel_,des_com_acc_);

    // =========================================================================
    // Torso Ori Task
    // =========================================================================
    //Eigen::VectorXd des_torso_quat = Eigen::VectorXd::Zero(4);
    //des_torso_quat << ini_torso_quat.w(),ini_torso_quat.x(), ini_torso_quat.y(),
                        //ini_torso_quat.z();
    //Eigen::VectorXd des_torso_so3 = Eigen::VectorXd::Zero(3);
    //Eigen::VectorXd des_ang_acc = Eigen::VectorXd::Zero(3);
    //torso_ori_task_ -> updateTask(des_torso_quat,des_torso_so3,des_ang_acc);
    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Draco::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(com_task_);
    //task_list_.push_back(torso_ori_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);
}

void BalanceCtrl::_contact_setup() {
    //rfoot_contact_->updateContactSpec();
    //lfoot_contact_->updateContactSpec();

    //contact_list_.push_back(rfoot_contact_);
    //contact_list_.push_back(lfoot_contact_);

    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);
}

void BalanceCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
    ctrl_start_time_ = sp_->curr_time;

    // ini_com_pos setting
    //Eigen::Quaternion<double> com_pos_ori_ini(robot_->getBodyNodeIsometry(DracoBodyNode::pelvis).linear());
    //ini_com_pos_ <<  com_pos_ori_ini.w(), com_pos_ori_ini.x(),
                    //com_pos_ori_ini.y(), com_pos_ori_ini.z();

    ini_com_pos_ = robot_ ->getCoMPosition();
    des_com_pos_ = ini_com_pos_ + com_pos_dev_;

    _SetBspline(ini_com_pos_, des_com_pos_);
    //myUtils::pretty_print(ini_com_pos_, std::cout, "ini");
    
    //ini_torso_quat = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(DracoBodyNode::torso).linear());
}

void BalanceCtrl::lastVisit() {}

bool BalanceCtrl::endOfPhase() {
     if (state_machine_time_ > end_time_) {
     return false;
    }
    return false;
}

void BalanceCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
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
    //fin[5] = amplitude_[] * omega_;
    com_traj_.SetParam(init, fin, middle_pt, end_time_);

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
        des_com_pos_[i] = pos[i];
        des_com_vel_[i] = vel[i];
        des_com_acc_[i] = acc[i];
    }
}
