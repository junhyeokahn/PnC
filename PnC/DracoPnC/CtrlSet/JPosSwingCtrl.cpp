#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/WBC/WBDC/WBDC.hpp>
#include <Utils/IO/DataManager.hpp>

JPosSwingCtrl::JPosSwingCtrl(RobotSystem* _robot):Controller(_robot) {
    myUtils::pretty_constructor(2, "JPos Ctrl");

    end_time_ = 1000.;
    b_jpos_set_ = false;
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    ctrl_start_time_ = 0.;

    set_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    amp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    freq_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    phase_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    jpos_task_ = new BasicTask(robot_, BasicTaskType::JOINT, robot_->getNumActuatedDofs());
    fixed_body_contact_ = new FixedBodyContactSpec(robot_);
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for(int i(0); i<robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();
    wbdc_data_->cost_weight = 
        Eigen::VectorXd::Constant(fixed_body_contact_->getDim() + 
                jpos_task_->getDim(), 100.0);
    wbdc_data_->cost_weight.tail(fixed_body_contact_->getDim()) = 
        Eigen::VectorXd::Constant(fixed_body_contact_->getDim(), 0.1);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

JPosSwingCtrl::~JPosSwingCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wbdc_;
    delete wbdc_data_;
}

void JPosSwingCtrl::oneStep(void* _cmd){
    _PreProcessing_Command();
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    _fixed_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    double ramp_time(2.0);
    if (state_machine_time_<ramp_time) {
        des_jvel_ *= state_machine_time_ / ramp_time;
        gamma *= state_machine_time_ / ramp_time;
    }

    ((DracoCommand*)_cmd)->jtrq = gamma;
    ((DracoCommand*)_cmd)->q = des_jpos_;
    ((DracoCommand*)_cmd)->qdot = des_jvel_;

    _PostProcessing_Command();
}

void JPosSwingCtrl::_jpos_ctrl_wbdc_rotor(Eigen::VectorXd & gamma){
    // Add rotor inertia
    //static int count__(0);
    //count__ ++;
    //if (count__ % 100 == 0) {
        //myUtils::pretty_print(A_, std::cout, "mass matrix before adding rotor inertia");
    //}
    for (int i = 0; i < robot_->getNumActuatedDofs(); ++i) {
        A_(i + robot_->getNumVirtualDofs(), i + robot_->getNumVirtualDofs())
            += sp_->rotor_inertia[i];
    }
    wbdc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->makeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void JPosSwingCtrl::_jpos_task_setup(){
    Eigen::VectorXd jacc_des(robot_->getNumActuatedDofs()); jacc_des.setZero();

    double ramp_value(0.);
    double ramp_duration(0.5);

    if(state_machine_time_ < ramp_duration){
        ramp_value = state_machine_time_/ramp_duration;
    }else{
        ramp_value = 1.;
    }

    double omega;

    for(int i(0); i<robot_->getNumActuatedDofs(); ++i){
        omega = 2. * M_PI * freq_[i];
        if(b_jpos_set_)
            des_jpos_[i] = set_jpos_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);
        else
            des_jpos_[i] = jpos_ini_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);

        des_jvel_[i] = amp_[i] * omega * cos(omega * state_machine_time_ + phase_[i]);
        des_jvel_[i] *= ramp_value;
        jacc_des[i] = -amp_[i] * omega * omega * sin(omega * state_machine_time_ + phase_[i]);
        jacc_des[i] *= ramp_value;
    }
    jpos_task_->updateTask(des_jpos_, des_jvel_, jacc_des);
    task_list_.push_back(jpos_task_);
}

void JPosSwingCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->updateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosSwingCtrl::firstVisit(){
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(), robot_->getNumActuatedDofs());
    ctrl_start_time_ = sp_->curr_time;
}

void JPosSwingCtrl::lastVisit(){  }

bool JPosSwingCtrl::endOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

void JPosSwingCtrl::ctrlInitialization(const YAML::Node& node){
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(), robot_->getNumActuatedDofs());

    try {
        Eigen::VectorXd kp, kd;
        myUtils::readParameter(node, "kp", kp);
        myUtils::readParameter(node, "kd", kd);
        jpos_task_->setGain(kp, kd);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
