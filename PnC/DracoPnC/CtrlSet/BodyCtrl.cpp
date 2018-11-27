#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/DataManager.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>

BodyCtrl::BodyCtrl(RobotSystem* robot) : Controller(robot) {

    end_time_ = 1000.;
    ctrl_start_time_ = 0.;
    b_set_height_target_ = false;
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    selected_jidx_.clear();
    selected_jidx_.push_back(robot_->getJointIdx("rHipYaw"));
    selected_jidx_.push_back(robot_->getJointIdx("lHipYaw"));
    selected_joint_task_ = new SelectedJointTask(robot_, selected_jidx_);

    body_rpz_task_ = new BodyRPZTask(robot);

    rfoot_contact_ = new PointContact(robot_, "rAnkle", 30);
    lfoot_contact_ = new PointContact(robot_, "lAnkle", 30);
    //rfoot_contact_ = new LineContact(robot_, "rAnkle", 3, 3);
    //lfoot_contact_ = new LineContact(robot_, "lAnkle", 3, 3);
    //rfoot_contact_ = new RectangularContactSpec(robot_, "rAnkle", 5);
    //lfoot_contact_ = new RectangularContactSpec(robot_, "lAnkle", 5);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for(int i(0); i<robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);

    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(robot_->getNumDofs(), 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    sp_ = DracoStateProvider::getStateProvider(robot_);

    printf("[[Body RPZ Ctrl]] Constructed\n");
}

BodyCtrl::~BodyCtrl(){
    delete body_rpz_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BodyCtrl::oneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    _double_contact_setup();
    _body_task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<robot_->getNumActuatedDofs(); ++i){
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyCtrl::_compute_torque_wblc(Eigen::VectorXd & gamma){
    // WBLC
    Eigen::MatrixXd A_rotor = A_;
    for (int i(0); i<robot_->getNumActuatedDofs(); ++i){
        A_rotor(i + robot_->getNumVirtualDofs(), i + robot_->getNumVirtualDofs())
            += sp_->rotor_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();

    std::vector<Task*> dummy_task_list_;

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

void BodyCtrl::_body_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    Eigen::VectorXd jpos_des(2); jpos_des.setZero();
    Eigen::VectorXd jvel_des(2); jvel_des.setZero();
    Eigen::VectorXd jacc_des(2); jacc_des.setZero();

    selected_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // Calculate IK for a desired height and orientation.
    double body_height_cmd;

    // Set Desired Orientation
    Eigen::Quaternion<double> des_quat( 1, 0, 0, 0 );

    Eigen::VectorXd pos_des(7); pos_des.setZero();
    Eigen::VectorXd vel_des(6); vel_des.setZero();
    Eigen::VectorXd acc_des(6); acc_des.setZero();

    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_height_;

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

    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(body_rpz_task_);

    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_,
            des_jpos_, des_jvel_, des_jacc_);

    //myUtils::pretty_print(jpos_ini_, std::cout, "jpos ini");
    //myUtils::pretty_print(des_jpos_, std::cout, "des jpos");
    //myUtils::pretty_print(des_jvel_, std::cout, "des jvel");
    //myUtils::pretty_print(des_jacc_, std::cout, "des jacc");
    //
    //Eigen::Vector3d com_pos;
    //robot_->getCoMPosition(com_pos);
    //dynacore::pretty_print(com_pos, std::cout, "com_pos");
}

void BodyCtrl::_double_contact_setup(){
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyCtrl::firstVisit(){
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(), robot_->getNumActuatedDofs());
    ctrl_start_time_ = sp_->curr_time;

    ini_body_pos_ = robot_->getQ().head(3);
    //ini_body_pos_ = robot_->getBodyNodeIsometry("torso").translation();
    //ini_body_pos_ = robot_->getBodyNodeCoMIsometry("torso").translation();
}

void BodyCtrl::lastVisit(){  }

bool BodyCtrl::endOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void BodyCtrl::ctrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(), robot_->getNumActuatedDofs());
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/CTRL/"+setting_file_name+".yaml");
        myUtils::readParameter(cfg, "kp", Kp_);
        myUtils::readParameter(cfg, "kd", Kd_);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
