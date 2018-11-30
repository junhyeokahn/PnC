#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/DataManager.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>

KinBalancingCtrl::KinBalancingCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "Kin Balancing Ctrl");

    end_time_ = 1000.;
    ctrl_start_time_ = 0.;
    interpolation_dur_ = 1.;

    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // task
    task_list_.clear();
    com_task_ = new BasicTask(robot, BasicTaskType::COM, 3);
    task_list_.push_back(com_task_);
    torso_ori_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3, "torso");
    task_list_.push_back(torso_ori_task_);

    // contact
    rfoot_front_contact_ = new PointContactSpec(robot_, "rFootFront", 3);
    rfoot_back_contact_ = new PointContactSpec(robot_, "rFootBack", 3);
    lfoot_front_contact_ = new PointContactSpec(robot_, "lFootFront", 3);
    lfoot_back_contact_ = new PointContactSpec(robot_, "lFootBack", 3);
    contact_list_.clear();
    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    fz_idx_in_cost_.clear();
    dim_contact_ = 0;
    for (int i = 0; i < contact_list_.size(); ++i) {
        fz_idx_in_cost_.push_back(dim_contact_ + contact_list_[i]->getFzIndex());
        dim_contact_ += contact_list_[i]->getDim();
    }

    // wbc
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for(int i(0); i<robot_->getNumVirtualDofs(); ++i) act_list[i] = false;
    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);

    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(robot_->getNumDofs(), 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    for (int i = 0; i < contact_list_.size(); ++i) {
        wblc_data_->W_rf_[fz_idx_in_cost_[i]] = 0.01;
    }
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

KinBalancingCtrl::~KinBalancingCtrl(){
    delete com_task_;
    delete torso_ori_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;

}

void KinBalancingCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    _contact_setup();
    _task_setup();
    _compute_torque(gamma);

    for(int i(0); i<robot_->getNumActuatedDofs(); ++i){
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void KinBalancingCtrl::_compute_torque(Eigen::VectorXd & gamma) {
    // WBLC
    Eigen::MatrixXd A_rotor = A_;
    for (int i(0); i<robot_->getNumActuatedDofs(); ++i){
        A_rotor(i + robot_->getNumVirtualDofs(), i + robot_->getNumVirtualDofs())
            += sp_->rotor_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();

    wblc_->updateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd = des_jacc_
        + Kp_.cwiseProduct(des_jpos_ -
                sp_->q.segment(robot_->getNumVirtualDofs(), robot_->getNumActuatedDofs()))
        + Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(robot_->getNumActuatedDofs()));

    wblc_->makeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    sp_->qddot_cmd = wblc_data_->qddot_;
    sp_->reaction_forces = wblc_data_->Fr_;

}

void KinBalancingCtrl::_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);

    com_pos_des = goal_com_pos_;
    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);
    task_list_.push_back(com_task_);

    Eigen::VectorXd torso_ori_pos_des = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd torso_ori_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd torso_ori_acc_des = Eigen::VectorXd::Zero(3);
    Eigen::Quaternion<double> des_quat( 1, 0, 0, 0 );

    torso_ori_pos_des[0] = des_quat.w();
    torso_ori_pos_des[1] = des_quat.x();
    torso_ori_pos_des[2] = des_quat.y();
    torso_ori_pos_des[3] = des_quat.z();
    torso_ori_task_->updateTask(torso_ori_pos_des, torso_ori_vel_des, torso_ori_acc_des);
    task_list_.push_back(torso_ori_task_);

    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_,
            des_jpos_, des_jvel_, des_jacc_);

    //myUtils::pretty_print(des_jpos_, std::cout, "des_jpos");
    //Eigen::VectorXd act_jpos = (sp_->q).tail(robot_->getNumActuatedDofs());
    //myUtils::pretty_print(act_jpos, std::cout, "act_jpos");
}

void KinBalancingCtrl::_contact_setup(){
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);
}

void KinBalancingCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(), robot_->getNumActuatedDofs());
    ctrl_start_time_ = sp_->curr_time;
    ini_com_pos_ = robot_->getCoMPosition();

    Eigen::VectorXd rfoot_pos = robot_->getBodyNodeIsometry("rAnkle").translation();
    Eigen::VectorXd lfoot_pos = robot_->getBodyNodeIsometry("lAnkle").translation();
    // TODO
    goal_com_pos_ = (rfoot_pos + lfoot_pos) / 2.0;
    goal_com_pos_[2] = ini_com_pos_[2];
    //goal_com_pos_[2] = ini_com_pos_[2] - 0.05;
    //goal_com_pos_ = ini_com_pos_;
    myUtils::pretty_print(rfoot_pos , std::cout, "rfoot_pos");
    myUtils::pretty_print(lfoot_pos , std::cout, "lfoot_pos");
    myUtils::pretty_print(ini_com_pos_, std::cout, "ini_com");
    myUtils::pretty_print(goal_com_pos_, std::cout, "goal_com");
}

void KinBalancingCtrl::lastVisit(){  }

bool KinBalancingCtrl::endOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

void KinBalancingCtrl::ctrlInitialization(const std::string & setting_file_name){
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/CTRL/"+setting_file_name+".yaml");
        myUtils::readParameter(cfg, "kp", Kp_);
        myUtils::readParameter(cfg, "kd", Kd_);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
