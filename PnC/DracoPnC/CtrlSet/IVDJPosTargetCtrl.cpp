#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <PnC/WBC/WBDC/WBDC.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
    
IVDJPosTargetCtrl::IVDJPosTargetCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "JPos Target Ctrl");

    jpos_target_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    end_time_ = 1000.0;
    ctrl_time_ = 1000.0;
    ctrl_start_time_ = 0.0;
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // task
    jpos_task_ = new BasicTask(robot_, BasicTaskType::JOINT,
                               robot_->getNumActuatedDofs());

    // contact
    // fixed_body_contact_ = new FixedBodyContactSpec(robot_);

    // wbc
    // std::vector<bool> act_list;
    // act_list.resize(robot_->getNumDofs(), true);
    // for (int i(0); i < robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    // wbdc_ = new WBDC(act_list);
    // wbdc_data_ = new WBDC_ExtraData();
    // wbdc_data_->cost_weight = Eigen::VectorXd::Constant(
    //     fixed_body_contact_->getDim() + jpos_task_->getDim(), 100.0);

    // wbdc_data_->cost_weight.tail(fixed_body_contact_->getDim()) =
    //     Eigen::VectorXd::Constant(fixed_body_contact_->getDim(), 0.1);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

IVDJPosTargetCtrl::~IVDJPosTargetCtrl() {
    delete jpos_task_;

    // delete fixed_body_contact_;

    // delete wbdc_;
    // delete wbdc_data_;
}

void IVDJPosTargetCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma;
    // _fixed_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc(gamma);

    double ramp_period(0.5);//(10);
    double ramp(1.0);
    if (state_machine_time_ < ramp_period)
        ramp = state_machine_time_ / ramp_period;

    ((DracoCommand*)_cmd)->jtrq = ramp * gamma;
    ((DracoCommand*)_cmd)->q = des_jpos_;
    ((DracoCommand*)_cmd)->qdot = des_jvel_;
    _PostProcessing_Command();
}

void IVDJPosTargetCtrl::_jpos_ctrl_wbdc(Eigen::VectorXd& gamma) {
    // std::cout << "num dofs:" << robot_->getNumDofs() << std::endl;
    // std::cout << "num act:" << robot_->getNumActuatedDofs() << std::endl;

    Eigen::VectorXd joint_task_cmd;
    Eigen::VectorXd qddot_des(robot_->getNumDofs());
    qddot_des.setZero();

    jpos_task_->getCommand(joint_task_cmd);
    qddot_des.tail(robot_->getNumActuatedDofs()) = joint_task_cmd;

    gamma = (A_*qddot_des + coriolis_ + grav_).tail( robot_->getNumActuatedDofs() );

    Eigen::VectorXd grav_tail = grav_.tail( robot_->getNumActuatedDofs() );
    myUtils::pretty_print(gamma, std::cout, "gamma");    
    myUtils::pretty_print(grav_tail, std::cout, "grav_tail");    

    // wbdc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    // wbdc_->makeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void IVDJPosTargetCtrl::_jpos_task_setup() {
    Eigen::VectorXd jacc_des(robot_->getNumActuatedDofs());
    jacc_des.setZero();

    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        des_jpos_[i] = myUtils::smooth_changing(jpos_ini_[i], jpos_target_[i],
                                                end_time_, state_machine_time_);
        des_jvel_[i] = myUtils::smooth_changing_vel(
            jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        jacc_des[i] = myUtils::smooth_changing_acc(
            jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
    }

    jpos_task_->updateTask(des_jpos_, des_jvel_, jacc_des);
    // task_list_.push_back(jpos_task_);
}

// void IVDJPosTargetCtrl::_fixed_body_contact_setup() {
//     fixed_body_contact_->updateContactSpec();
//     contact_list_.push_back(fixed_body_contact_);
// }

void IVDJPosTargetCtrl::firstVisit() {
    ctrl_start_time_ = sp_->curr_time;
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
}

void IVDJPosTargetCtrl::lastVisit() {}

bool IVDJPosTargetCtrl::endOfPhase() {
    if (state_machine_time_ > ctrl_time_) {
        return true;
    }
    return false;
}

void IVDJPosTargetCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    try {
        Eigen::VectorXd kp, kd;
        myUtils::readParameter(node, "kp", kp);
        myUtils::readParameter(node, "kd", kd);
        jpos_task_->setGain(kp, kd);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void IVDJPosTargetCtrl::setTargetPosition(const Eigen::VectorXd& jpos) {
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        jpos_target_[i] = jpos[i];
    }
}
