#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <Configuration.h>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <Utils/Utilities.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>

SingleContactTransCtrl::SingleContactTransCtrl(RobotSystem* robot,
        std::string moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.),
    des_jpos_(robot_->getNumActuatedDofs()),
    des_jvel_(robot_->getNumActuatedDofs()),
    des_jacc_(robot_->getNumActuatedDofs()),
    Kp_(robot_->getNumActuatedDofs()),
    Kd_(robot_->getNumActuatedDofs())
{
    myUtils::pretty_constructor(2, "Single Contact Trans Ctrl");

    base_task_ = new BodyRPZTask(robot);

    selected_jidx_.clear();
    selected_jidx_.push_back(robot_->getJointIdx("rHipYaw"));
    selected_jidx_.push_back(robot_->getJointIdx("lHipYaw"));

    selected_joint_task_ = new SelectedJointTask(robot_, selected_jidx_);

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
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

SingleContactTransCtrl::~SingleContactTransCtrl(){
    delete base_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;
}

void SingleContactTransCtrl::oneStep(void* _cmd){
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

void SingleContactTransCtrl::_compute_torque_wblc(Eigen::VectorXd & gamma){
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

    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    //dynacore::pretty_print(gamma, std::cout, "gamma");

    sp_->qddot_cmd = wblc_data_->qddot_;
    sp_->reaction_forces = wblc_data_->Fr_;
}

void SingleContactTransCtrl::_task_setup(){
    // Body height
    double base_height_cmd = ini_base_height_;
    if(b_set_height_target_) base_height_cmd = des_base_height_;

    Eigen::VectorXd jpos_des(2); jpos_des.setZero();
    Eigen::VectorXd jvel_des(2); jvel_des.setZero();
    Eigen::VectorXd jacc_des(2); jacc_des.setZero();

    selected_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // Orientation
    Eigen::Quaternion<double> des_quat(1, 0, 0, 0);

    Eigen::VectorXd pos_des(7); pos_des.setZero();
    Eigen::VectorXd vel_des(6); vel_des.setZero();
    Eigen::VectorXd acc_des(6); acc_des.setZero();

    pos_des[0] = des_quat.w();
    pos_des[1] = des_quat.x();
    pos_des[2] = des_quat.y();
    pos_des[3] = des_quat.z();

    pos_des[4] = 0.;
    pos_des[5] = ini_base_pos_[1];
    pos_des[6] = base_height_cmd;

    base_task_->updateTask(pos_des, vel_des, acc_des);

    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(base_task_);

    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_,
            des_jpos_, des_jvel_, des_jacc_);

    // TEST
    if(b_increase_){
        if(moving_foot_ == "rAnkle"){
            int swing_jidx = robot_->getJointIdx("rHipYaw") - robot_->getNumVirtualDofs();
            double h(state_machine_time_/end_time_);

            for(int i(0); i < 5; ++i){
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] +=
                    (1. - h)*sp_->des_jpos_prev[swing_jidx + i];
             }
        }
        else if(moving_foot_ == "lAnkle"){
            int swing_jidx = robot_->getJointIdx("lHipYaw") - robot_->getNumVirtualDofs();
            double h(state_machine_time_/end_time_);

            for(int i(0); i < 5; ++i){
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] +=
                    (1. - h)*sp_->des_jpos_prev[swing_jidx + i];
            }
        }
    }
}

void SingleContactTransCtrl::_contact_setup(){
    double alpha = 0.5 * (1-cos(M_PI * state_machine_time_/end_time_));
    double upper_lim(100.);
    double rf_weight(100.);
    double rf_weight_z(100.);
    double foot_weight(1000.);

    if(b_increase_){
        upper_lim = min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;
        foot_weight = 0.001 * (1. - alpha)  + 1000. * alpha;
    } else {
        upper_lim = max_rf_z_ - alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (alpha) * 5. + (1. - alpha) * 1.0;
        rf_weight_z = (alpha) * 0.5 + (1. - alpha) * 0.01;
        foot_weight = 0.001 * (alpha)  + 1000. * (1. - alpha);
    }
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    int jidx_offset(0);
    if(moving_foot_ == "lAnkle") {
        jidx_offset = rfoot_contact_->getDim();
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] = rf_weight_z;

        ((PointContact*)lfoot_contact_)->setMaxFz(upper_lim);
    }
    else if(moving_foot_ == "rAnkle") {
        for(int i(0); i<rfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] = rf_weight_z;

        ((PointContact*)rfoot_contact_)->setMaxFz(upper_lim);
    }
}

void SingleContactTransCtrl::firstVisit(){
    // printf("[Transition] Start\n");
    ctrl_start_time_ = sp_->curr_time;

    //ini_base_pos_ = robot_->getBodyNodeCoMIsometry("torso").translation();
    ini_base_pos_ = sp_->q.head(3);
}

void SingleContactTransCtrl::lastVisit(){
    sp_->des_jpos_prev = des_jpos_;
    // printf("[Transition] End\n");
}

bool SingleContactTransCtrl::endOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void SingleContactTransCtrl::ctrlInitialization(const std::string & setting_file_name){
    ini_base_height_ = robot_->getBodyNodeCoMIsometry("torso").translation()[2];
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
