#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <Configuration.h>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/PlannerSet/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/Utilities.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>

BodyFootPlanningCtrl::BodyFootPlanningCtrl(RobotSystem* robot,
                                           std::string swing_foot,
                                           FootStepPlanner* planner)
    : SwingPlanningCtrl(robot, swing_foot, planner) {

    push_down_height_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    waiting_time_limit_ = 0.02;
    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    rfoot_contact_ = new PointContact(robot_, "rAnkle", 30);
    lfoot_contact_ = new PointContact(robot_, "lAnkle", 30);
    //rfoot_contact_ = new LineContact(robot_, "rAnkle", 3, 3);
    //lfoot_contact_ = new LineContact(robot_, "lAnkle", 3, 3);
    //rfoot_contact_ = new RectangularContactSpec(robot_, "rAnkle", 5);
    //lfoot_contact_ = new RectangularContactSpec(robot_, "lAnkle", 5);

    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    selected_jidx_.clear();
    selected_jidx_.push_back(robot_->getJointIdx("rHipYaw"));
    selected_jidx_.push_back(robot_->getJointIdx("lHipYaw"));
    selected_joint_task_ = new SelectedJointTask(robot_, selected_jidx_);

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

    kin_wbc_contact_list_.clear();
    int jidx_offset(0);
    if(swing_foot == "lAnkle") {
        jidx_offset = rfoot_contact_->getDim();
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = 5.0;
            wblc_data_->W_xddot_[i + jidx_offset] = 0.001;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] = 0.5;

        ((PointContact*)lfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(rfoot_contact_);
    }
    else if(swing_foot == "rAnkle") {
        for(int i(0); i<rfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + jidx_offset] = 5.0;
            wblc_data_->W_xddot_[i + jidx_offset] = 0.0001;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] = 0.5;

        ((PointContact*)rfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(lfoot_contact_);
    }

    foot_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, swing_foot);
    base_task_ = new BodyRPZTask(robot_);

    // Create Minimum jerk objects
    for(size_t i = 0; i < 3; i++){
        min_jerk_offset_.push_back(new MinJerk_OneDimension());
    }
    printf("[Body Foot Planning Controller] Constructed\n");
}

void BodyFootPlanningCtrl::oneStep(void* _cmd){
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

void BodyFootPlanningCtrl::_contact_setup(){
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyFootPlanningCtrl::_compute_torque_wblc(Eigen::VectorXd & gamma){
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

    //myUtils::pretty_print(des_jpos_, std::cout, "des_jpos");
    //myUtils::pretty_print(des_jvel_, std::cout, "des_jvel");
    //myUtils::pretty_print(des_jacc_, std::cout, "des_jacc");
    //myUtils::pretty_print(des_jacc_cmd, std::cout, "des_jacc");

    wblc_->makeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    sp_->qddot_cmd = wblc_data_->qddot_;
    sp_->reaction_forces = wblc_data_->Fr_;
}

void BodyFootPlanningCtrl::_task_setup(){
    // Body height
    double base_height_cmd = ini_base_height_;
    if(b_set_height_target_) base_height_cmd = des_body_height_;

    Eigen::VectorXd jpos_des(2); jpos_des.setZero();
    Eigen::VectorXd jvel_des(2); jvel_des.setZero();
    Eigen::VectorXd jacc_des(2); jacc_des.setZero();

    selected_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // Orientation
    Eigen::Quaternion<double> des_quat(1, 0, 0, 0);
    /////// Body Posture Task Setup
    Eigen::VectorXd pos_des(7); pos_des.setZero();
    Eigen::VectorXd vel_des(6); vel_des.setZero();
    Eigen::VectorXd acc_des(6); acc_des.setZero();

    pos_des[0] = des_quat.w();
    pos_des[1] = des_quat.x();
    pos_des[2] = des_quat.y();
    pos_des[3] = des_quat.z();

    pos_des[4] = 0.;
    pos_des[5] = ini_body_pos_[1];
    pos_des[6] = base_height_cmd;

    base_task_->updateTask(pos_des, vel_des, acc_des);

    /////// Foot Pos Task Setup
    _CheckPlanning();
    //_GetSinusoidalSwingTrajectory();
    _GetBsplineSwingTrajectory();

    double traj_time = state_machine_time_ - half_swing_time_;
    if(state_machine_time_ > half_swing_time_){
        double pos, vel, acc;
        for(int i(0); i<3; ++i){
            min_jerk_offset_[i]->getPos(traj_time, pos);
            min_jerk_offset_[i]->getVel(traj_time, vel);
            min_jerk_offset_[i]->getAcc(traj_time, acc);

            curr_foot_pos_des_[i] += pos;
            curr_foot_vel_des_[i] += vel;
            curr_foot_acc_des_[i] += acc;
        }
    }
    if(state_machine_time_> end_time_){
        for(int i(0); i<foot_task_->getDim(); ++i){
            curr_foot_vel_des_[i] = 0;
            curr_foot_acc_des_[i] = 0;
        }
        curr_foot_pos_des_[2] =
            -push_down_height_ - 0.1*(state_machine_time_ - end_time_);
    }

    Eigen::VectorXd foot_pos_des(foot_task_->getDim()); foot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(foot_task_->getDim()); foot_vel_des.setZero();
    Eigen::VectorXd foot_acc_des(foot_task_->getDim()); foot_acc_des.setZero();

    foot_pos_des = curr_foot_pos_des_;
    foot_vel_des = curr_foot_vel_des_;
    foot_acc_des = curr_foot_acc_des_;

    foot_task_->updateTask(
            foot_pos_des,
            foot_vel_des,
            foot_acc_des);

    task_list_.push_back(selected_joint_task_);
    task_list_.push_back(base_task_);
    task_list_.push_back(foot_task_);

    kin_wbc_->FindConfiguration(sp_->q, task_list_, kin_wbc_contact_list_,
            des_jpos_, des_jvel_, des_jacc_);
}

void BodyFootPlanningCtrl::_CheckPlanning(){
    if( (state_machine_time_ > 0.5 * end_time_) && b_replanning_ && !b_replaned_) {

        Eigen::Vector3d target_loc;
        _Replanning(target_loc);

        Eigen::Vector3d target_offset;
        // X, Y target is originally set by intial_traget_loc
        for(int i(0); i<2; ++i)
            target_offset[i] = target_loc[i] - initial_target_loc_[i];

        // Foot height (z) is set by the initial height
        target_offset[2] = 0.; //target_loc[2] - ini_foot_pos_[2];

        _SetMinJerkOffset(target_offset);
        b_replaned_ = true;
    }
}

void BodyFootPlanningCtrl::_Replanning(Eigen::Vector3d & target_loc){
    // Direct value used
    Eigen::Vector3d com_pos = robot_->getCoMPosition();
    Eigen::Vector3d com_vel = robot_->getCoMVelocity();

    // TEST
    for(int i(0); i<2; ++i){
        com_pos[i] = sp_->q[i] + body_pt_offset_[i];
        com_vel[i] = sp_->qdot[i];
    }

    printf("planning com state: %f, %f, %f, %f\n",
            com_pos[0], com_pos[1],
            com_vel[0], com_vel[1]);

    OutputReversalPL pl_output;
    ParamReversalPL pl_param;
    pl_param.swing_time = end_time_ - state_machine_time_
        + transition_time_ * transition_phase_ratio_
        + stance_time_ * double_stance_ratio_;


    pl_param.des_loc = sp_->des_location;
    pl_param.stance_foot_loc = sp_->global_pos_local;

    if(swing_foot_ == "lAnkle")
        pl_param.b_positive_sidestep = true;
    else
        pl_param.b_positive_sidestep = false;


    Eigen::Vector3d global_com_pos = com_pos + sp_->global_pos_local;
    //myUtils::pretty_print(global_com_pos, std::cout, "***planning com pos global");
    //myUtils::pretty_print(com_vel, std::cout, "***planning com vel global");

    planner_->getNextFootLocation(global_com_pos,
            com_vel,
            target_loc,
            &pl_param, &pl_output);

    //myUtils::pretty_print(target_loc, std::cout, "***next foot global");
    Eigen::VectorXd ss_global(4);
    for (int i = 0; i < 4; ++i) {
        ss_global[i] = pl_output.switching_state[i];
    }
    //myUtils::pretty_print(ss_global, std::cout, "***planned ss global");
    // Time Modification
    replan_moment_ = state_machine_time_;
    end_time_ += pl_output.time_modification;
    target_loc -= sp_->global_pos_local;

    target_loc[2] = initial_target_loc_[2];

    // TEST
    for(int i(0); i<2; ++i){
        target_loc[i] += foot_landing_offset_[i];
    }
    myUtils::pretty_print(target_loc, std::cout, "next foot loc");
}

void BodyFootPlanningCtrl::firstVisit(){
    b_replaned_ = false;
    ini_config_ = sp_->q;
    //ini_body_pos_ = robot_->getBodyNodeCoMIsometry("torso").translation();
    ini_body_pos_ = sp_->q.head(3);
    ini_foot_pos_ = robot_->getBodyNodeCoMIsometry(swing_foot_).translation();
    ctrl_start_time_ = sp_->curr_time;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    initial_target_loc_[0] = sp_->q[0];
    initial_target_loc_[1] = sp_->q[1] + default_target_loc_[1];
    initial_target_loc_[2] = -push_down_height_;

    _SetBspline(ini_foot_pos_, initial_target_loc_);

    Eigen::Vector3d foot_pos_offset; foot_pos_offset.setZero();
    foot_pos_offset[2] = 0.;
    _SetMinJerkOffset(foot_pos_offset);

    Eigen::Vector3d ini_com_pos_ = robot_->getCoMPosition();
    Eigen::Vector3d com_vel = robot_->getCoMVelocity();

    Eigen::VectorXd input_state(4);
    input_state[0] = ini_com_pos_[0];
    input_state[1] = ini_com_pos_[1];
    input_state[2] = com_vel[0];   input_state[3] = com_vel[1];

    //TODO
    //myUtils::pretty_print(sp_->q, std::cout, "q");
    //myUtils::pretty_print(sp_->qdot, std::cout, "qdot");
    //exit(0);
    //TODO
}

void BodyFootPlanningCtrl::_SetMinJerkOffset(const Eigen::Vector3d & offset){
    // Initialize Minimum Jerk Parameter Containers
    Eigen::Vector3d init_params;
    Eigen::Vector3d final_params;

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < 3; i++){
        // Set Dimension i's initial pos, vel and acceleration
        init_params.setZero();
        // Set Dimension i's final pos, vel, acceleration
        final_params.setZero();
        final_params[0] = offset[i];

        min_jerk_offset_[i]->setParams(
                init_params, final_params,
                0., half_swing_time_);
    }
}

bool BodyFootPlanningCtrl::endOfPhase(){
    if(state_machine_time_ > (end_time_ + waiting_time_limit_)){
        printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n",
                state_machine_time_, end_time_);
        return true;
    }
    // Swing foot contact = END
    if(b_contact_switch_check_){
        bool contact_happen(false);
        if(swing_foot_ == "lAnkle" && sp_->b_lfoot_contact){
            contact_happen = true;
        }
        if(swing_foot_ == "rAnkle" && sp_->b_rfoot_contact){
            contact_happen = true;
        }
        if(state_machine_time_ > end_time_ * 0.5 && contact_happen){
            printf("[Config Body Foot Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
                    state_machine_time_, end_time_);
            return true;
        }
    }
    return false;
}

void BodyFootPlanningCtrl::ctrlInitialization(const std::string & setting_file_name){
    ini_base_height_ = sp_->q[2];
    //ini_base_height_ = (robot_->getBodyNodeCoMIsometry("torso").translation())[2];
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/CTRL/"+setting_file_name+".yaml");
        myUtils::readParameter(cfg, "kp", Kp_);
        myUtils::readParameter(cfg, "kd", Kd_);
        myUtils::readParameter(cfg, "swing_height", swing_height_);
        myUtils::readParameter(cfg, "push_down_height", push_down_height_);
        myUtils::readParameter(cfg, "default_target_foot_location", default_target_loc_);
        myUtils::readParameter(cfg, "body_pt_offset", body_pt_offset_);
        myUtils::readParameter(cfg, "foot_landing_offset", foot_landing_offset_);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
    static bool b_bodypute_eigenvalue(true);
    if(b_bodypute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ +
                    transition_phase_ratio_*transition_time_ +
                    end_time_);
        b_bodypute_eigenvalue = false;
    }
}

BodyFootPlanningCtrl::~BodyFootPlanningCtrl(){
    delete wblc_;
    delete lfoot_contact_;
    delete rfoot_contact_;
}

void BodyFootPlanningCtrl::_GetBsplineSwingTrajectory(){
    double pos[3];
    double vel[3];
    double acc[3];

    foot_traj_.getCurvePoint(state_machine_time_, pos);
    foot_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    foot_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for(int i(0); i<3; ++i){
        curr_foot_pos_des_[i] = pos[i];
        curr_foot_vel_des_[i] = vel[i];
        curr_foot_acc_des_[i] = acc[i];
    }
}
void BodyFootPlanningCtrl::_GetSinusoidalSwingTrajectory(){
    curr_foot_acc_des_.setZero();
    for (int i(0); i<2; ++i){
        curr_foot_pos_des_[i] =
            myUtils::smooth_changing(ini_foot_pos_[i], initial_target_loc_[i],
                    end_time_, state_machine_time_);
        curr_foot_vel_des_[i] =
            myUtils::smooth_changing_vel(ini_foot_pos_[i], initial_target_loc_[i],
                    end_time_, state_machine_time_);
        curr_foot_acc_des_[i] =
            myUtils::smooth_changing_acc(ini_foot_pos_[i], initial_target_loc_[i],
                    end_time_, state_machine_time_);
    }
    // for Z (height)
    double amp(swing_height_/2.);
    double omega ( 2.*M_PI /end_time_ );

    curr_foot_pos_des_[2] =
        ini_foot_pos_[2] + amp * (1-cos(omega * state_machine_time_));
    curr_foot_vel_des_[2] =
        amp * omega * sin(omega * state_machine_time_);
    curr_foot_acc_des_[2] =
        amp * omega * omega * cos(omega * state_machine_time_);
}

void BodyFootPlanningCtrl::_SetBspline(
        const Eigen::Vector3d & st_pos,
        const Eigen::Vector3d & des_pos){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    Eigen::Vector3d middle_pos;

    middle_pos = (st_pos + des_pos)/2.;
    middle_pos[2] = swing_height_;

    // Initial and final position & velocity & acceleration
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_pos[i];
        init[i+3] = 0.;
        init[i+6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i+3] = 0.;
        fin[i+6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    fin[5] = -0.5;
    fin[8] = 5.;
    foot_traj_.SetParam(init, fin, middle_pt, end_time_);

    delete [] *middle_pt;
    delete [] middle_pt;
}


