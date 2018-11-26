#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBDC/WBDC.hpp>
#include <Utils/DataManager.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>

BalancingCtrl::BalancingCtrl(RobotSystem* robot) : Controller(robot) {

    end_time_ = 1000.;
    ctrl_start_time_ = 0.;
    interpolation_dur_ = 1.;

    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    //contact
    rfoot_contact_ = new RectangleContactSpec(robot_, "rAnkle", 3.0);
    lfoot_contact_ = new RectangleContactSpec(robot_, "lAnkle", 3.0);
    //rfoot_contact_ = new PointContact(robot_, "rAnkle", 3.0);
    //lfoot_contact_ = new PointContact(robot_, "lAnkle", 3.0);
    contact_list_.clear();
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    centroid_task_ = new BasicTask(robot_, BasicTaskType::CENTROID, 6);
    task_list_.clear();
    task_list_.push_back(centroid_task_);
    centroid_pos_des_ = Eigen::VectorXd::Zero(centroid_task_->getDim());
    centroid_vel_des_ = Eigen::VectorXd::Zero(centroid_task_->getDim());
    centroid_acc_des_ = Eigen::VectorXd::Zero(centroid_task_->getDim());
    centroid_pos_act_ = Eigen::VectorXd::Zero(centroid_task_->getDim());
    centroid_vel_act_ = Eigen::VectorXd::Zero(centroid_task_->getDim());
    centroid_acc_act_ = Eigen::VectorXd::Zero(centroid_task_->getDim());

    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for(int i(0); i < robot_->getNumVirtualDofs(); ++i)
        act_list[i] = false;
    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();

    // cost setting
    int task_dim(0);
    int contact_dim(0);
    int prev_contact_dim(0);
    for (int i = 0; i < task_list_.size(); ++i) { task_dim += task_list_[i]->getDim(); }
    for (int i = 0; i < contact_list_.size(); ++i) {contact_dim += contact_list_[i]->getDim(); }
    wbdc_data_->cost_weight = Eigen::VectorXd::Zero(task_dim + contact_dim);

    for(int i(0); i<task_dim; ++i) { wbdc_data_->cost_weight[i] = 1000.0; }
    for(int i(0); i<contact_dim; ++i) {
        wbdc_data_->cost_weight[task_dim + i] = 1.0;
    }
    for(int i(0); i<contact_list_.size(); ++i) {
        wbdc_data_->cost_weight[task_dim + prev_contact_dim + contact_list_[i]->getFzIndex()] = 0.001;
        prev_contact_dim = contact_list_[i]->getDim();
    }

    sp_ = DracoStateProvider::getStateProvider(robot_);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&centroid_pos_des_, VECT, "centroid_pos_des", 6);
    data_manager->RegisterData(&centroid_vel_des_, VECT, "centroid_vel_des", 6);
    data_manager->RegisterData(&centroid_acc_des_, VECT, "centroid_acc_des", 6);
    data_manager->RegisterData(&centroid_pos_act_, VECT, "centroid_pos_act", 6);
    data_manager->RegisterData(&centroid_vel_act_, VECT, "centroid_vel_act", 6);
    data_manager->RegisterData(&centroid_acc_act_, VECT, "centroid_acc_act", 6);

    printf("[Balancing Ctrl] Constructed\n");
}

BalancingCtrl::~BalancingCtrl(){
    delete centroid_task_;
    delete wbdc_;
    delete wbdc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BalancingCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    _contact_setup();
    _task_setup();
    _compute_torque(gamma);

    for(int i(0); i<robot_->getNumActuatedDofs(); ++i){
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = sp_->q[i+6];
        ((DracoCommand*)_cmd)->qdot[i] = sp_->qdot[i+6];
    }
    _PostProcessing_Command();
}

void BalancingCtrl::_compute_torque(Eigen::VectorXd & gamma) {
    wbdc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->makeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void BalancingCtrl::_task_setup(){
    centroid_pos_des_.setZero();
    centroid_vel_des_.setZero();
    centroid_acc_des_.setZero();
    double d_ary[3];
    if (state_machine_time_ < interpolation_dur_) {
        for (int i = 0; i < 3; ++i) {
            centroid_pos_des_[i+3] = myUtils::smooth_changing(ini_com_pos_[i],
                    goal_com_pos_[i], interpolation_dur_, state_machine_time_);
            centroid_vel_des_[i+3] = myUtils::smooth_changing_vel(ini_com_pos_[i],
                    goal_com_pos_[i], interpolation_dur_, state_machine_time_);
            centroid_acc_des_[i+3] = myUtils::smooth_changing_acc(ini_com_pos_[i],
                    goal_com_pos_[i], interpolation_dur_, state_machine_time_);
        }
    } else {
        centroid_pos_des_.tail(3) = goal_com_pos_;
    }
    centroid_task_->updateTask(centroid_pos_des_, centroid_vel_des_, centroid_acc_des_);
    task_list_.push_back(centroid_task_);

    centroid_pos_act_.tail(3) = robot_->getCoMPosition();
    centroid_vel_act_ = robot_->getCentroidMomentum();
}

void BalancingCtrl::_contact_setup(){
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BalancingCtrl::firstVisit() {
    ctrl_start_time_ = sp_->curr_time;

    ini_com_pos_ = robot_->getCoMPosition();
    ini_com_vel_ = robot_->getCoMVelocity();

    Eigen::VectorXd rfoot_pos = robot_->getBodyNodeIsometry("rAnkle").translation();
    Eigen::VectorXd lfoot_pos = robot_->getBodyNodeIsometry("lAnkle").translation();

    // TODO
    goal_com_pos_ = (rfoot_pos + lfoot_pos) / 2.0;
    //goal_com_pos_[0] += 0.015;
    goal_com_pos_[2] = ini_com_pos_[2] - 0.05;
    //goal_com_pos_ = ini_com_pos_;
    myUtils::pretty_print(rfoot_pos , std::cout, "rfoot_pos");
    myUtils::pretty_print(lfoot_pos , std::cout, "lfoot_pos");
    myUtils::pretty_print(ini_com_pos_, std::cout, "ini_com");
    myUtils::pretty_print(goal_com_pos_, std::cout, "goal_com");
    goal_com_vel_ = Eigen::VectorXd::Zero(3);

    double ini[9] = {ini_com_pos_[0] , ini_com_pos_[1] , ini_com_pos_[2] ,
                     ini_com_vel_[0] , ini_com_vel_[1] , ini_com_vel_[2] ,
                     0               , 0               , 0               };
    double fin[9] = {goal_com_pos_[0] , goal_com_pos_[1] , goal_com_pos_[2] ,
                     goal_com_vel_[0] , goal_com_vel_[1] , goal_com_vel_[2] ,
                     0                , 0                , 0               };
    double **middle_pt;
    spline_.SetParam(ini, fin, middle_pt, interpolation_dur_);
}

void BalancingCtrl::lastVisit(){  }

bool BalancingCtrl::endOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

void BalancingCtrl::ctrlInitialization(const std::string & setting_file_name){
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/CTRL/"+setting_file_name+".yaml");
        myUtils::readParameter(cfg, "centroid_task_kp", Kp_);
        myUtils::readParameter(cfg, "centroid_task_kd", Kd_);
        centroid_task_->setGain(Kp_, Kd_);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
