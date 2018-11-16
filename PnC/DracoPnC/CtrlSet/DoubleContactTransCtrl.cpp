#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>

DoubleContactTransCtrl::DoubleContactTransCtrl(RobotSystem* robot) : Controller(robot) {

    b_set_height_target_ = false;
    end_time_ = 100.;
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    body_rpz_task_ = new BodyRPZTask(robot);

    selected_jidx_.clear();
    selected_jidx_.push_back(robot_->getJointIdx("rHipYaw"));
    selected_jidx_.push_back(robot_->getJointIdx("lHipYaw"));
    selected_joint_task_ = new SelectedJointTask(robot_, selected_jidx_);

    rfoot_contact_ = new PointContact(robot_, "rAnkle", 0.3);
    lfoot_contact_ = new PointContact(robot_, "lAnkle", 0.3);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for(int i(0); i<robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);

    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(robot_->getNumDofs(), 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    sp_ = DracoStateProvider::getStateProvider(robot_);

    printf("[Double Contact Transition Ctrl] Constructed\n");
}

DoubleContactTransCtrl::~DoubleContactTransCtrl(){
    delete body_rpz_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;
    delete wblc_;
    delete kin_wbc_;
    delete wblc_data_;
}

void DoubleContactTransCtrl::oneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma;
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<robot_->getNumActuatedDofs(); ++i){
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void DoubleContactTransCtrl::_compute_torque_wblc(Eigen::VectorXd & gamma){
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

void DoubleContactTransCtrl::_task_setup(){
    des_jpos_ = ini_jpos_;
    des_jvel_.setZero();
    des_jacc_.setZero();
     // Calculate IK for a desired height and orientation.
    double base_height_cmd;

    Eigen::VectorXd jpos_des(2); jpos_des.setZero();
    Eigen::VectorXd jvel_des(2); jvel_des.setZero();
    Eigen::VectorXd jacc_des(2); jacc_des.setZero();

    selected_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // Set Desired Orientation
    Eigen::Quaternion<double> des_quat( 1, 0, 0, 0 );

    Eigen::VectorXd pos_des(7); pos_des.setZero();
    Eigen::VectorXd vel_des(6); vel_des.setZero();
    Eigen::VectorXd acc_des(6); acc_des.setZero();
    if(b_set_height_target_) base_height_cmd = des_base_height_;
    else base_height_cmd = base_pos_ini_[2];

    if(b_set_height_target_){
        base_height_cmd =
            myUtils::smooth_changing(ini_base_height_, des_base_height_,
                    end_time_, state_machine_time_);
    }else{
        printf("[Warning] The body height is not specified\n");
    }

    pos_des[0] = des_quat.w();
    pos_des[1] = des_quat.x();
    pos_des[2] = des_quat.y();
    pos_des[3] = des_quat.z();

    pos_des[4] = base_pos_ini_[0];
    pos_des[5] = base_pos_ini_[1];
    pos_des[6] = base_height_cmd;

    body_rpz_task_->updateTask(pos_des, vel_des, acc_des);

    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(body_rpz_task_);

    kin_wbc_->Ainv_ = Ainv_;
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_,
            des_jpos_, des_jvel_, des_jacc_);
    //dynacore::pretty_print(sp_->q, std::cout, "curr_config");
    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
}

void DoubleContactTransCtrl::_contact_setup(){
    //((SingleContact*)rfoot_contact_)->setMaxFz(
        //min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );
    //((SingleContact*)lfoot_contact_)->setMaxFz(
        //min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );

    ((PointContact*)rfoot_contact_)->setMaxFz(
        min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );
    ((PointContact*)lfoot_contact_)->setMaxFz(
        min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );


    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void DoubleContactTransCtrl::firstVisit(){
    // TODO
    //myUtils::pretty_print(sp_->q, std::cout, "q");
    //myUtils::pretty_print(sp_->qdot, std::cout, "qdot");
    // TODO
    ini_base_height_ = robot_->getQ()[2];
    //ini_base_height_ = (robot_->getBodyNodeCoMIsometry("torso").translation())[2];
    ctrl_start_time_ = sp_->curr_time;
    base_pos_ini_ = robot_->getQ().head(3);
    //base_pos_ini_ = robot_->getBodyNodeCoMIsometry("torso").translation();
    base_ori_ini_ = robot_->getBodyNodeCoMIsometry("torso").linear();
}

void DoubleContactTransCtrl::lastVisit(){
}

bool DoubleContactTransCtrl::endOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void DoubleContactTransCtrl::ctrlInitialization(const std::string & setting_file_name){
    ini_jpos_ = sp_->q.segment(robot_->getNumVirtualDofs(), robot_->getNumActuatedDofs());
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/CTRL/"+setting_file_name+".yaml");
        myUtils::readParameter(cfg, "kp", Kp_);
        myUtils::readParameter(cfg, "kd", Kd_);
        myUtils::readParameter(cfg, "max_rf_z", max_rf_z_);
        myUtils::readParameter(cfg, "min_rf_z", min_rf_z_);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
