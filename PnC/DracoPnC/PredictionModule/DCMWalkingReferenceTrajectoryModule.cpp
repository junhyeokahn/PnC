#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>
#include <Utils/IO/IOUtilities.hpp>

// Initialize by assigning the contact indices to a robot side.
DCMWalkingReferenceTrajectoryModule::DCMWalkingReferenceTrajectoryModule(const std::vector<int> & index_to_side_in){

    t_walk_start_ = 0.0;
    // Initialize object pointers
    reaction_force_schedule_ptr.reset(new DCMWalkingReactionForceSchedule(this));
    setContactIndexToSide(index_to_side_in);

    //std::cout << "[DCMWalkingReferenceTrajectoryModule] Constructed" << std::endl;
}

DCMWalkingReferenceTrajectoryModule::~DCMWalkingReferenceTrajectoryModule(){

}

void DCMWalkingReferenceTrajectoryModule::setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in){
    t_walk_start_ = t_walk_start_in;
    // Clear internal data then copy footsteps
    footstep_list_.clear();
    early_contact_times_.clear();
    footstep_list_ = footstep_list_in;

    // Set DCM reference
	dcm_reference.setInitialTime(t_walk_start_);
	dcm_reference.setInitialOri(x_ori_start_);
	dcm_reference.initialize_footsteps_rvrp(footstep_list_, left_foot_start_,
            right_foot_start_, x_com_start_);
}

void DCMWalkingReferenceTrajectoryModule::initialize(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in,
		 											 const Eigen::Vector3d  dcm_pos_start_in,
		 											 const Eigen::Vector3d  dcm_vel_start_in,	
												     const Eigen::Quaterniond & ori_start_in,
												     const DracoFootstep & left_foot_start_in,
												     const DracoFootstep & right_foot_start_in){
    t_walk_start_ = t_walk_start_in;
    // Clear internal data then copy footsteps
    footstep_list_.clear();
    early_contact_times_.clear();
    footstep_list_ = footstep_list_in;

    x_ori_start_ = ori_start_in;
    left_foot_start_ = left_foot_start_in;
    right_foot_start_ = right_foot_start_in;

    // Set DCM reference
	dcm_reference.setInitialTime(t_walk_start_);
	dcm_reference.setInitialOri(x_ori_start_);
	dcm_reference.initialize_footsteps_rvrp(footstep_list_, left_foot_start_,
            right_foot_start_, dcm_pos_start_in, dcm_vel_start_in);
}

void DCMWalkingReferenceTrajectoryModule::getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out){

}

void DCMWalkingReferenceTrajectoryModule::getMPCRefQuatAngVelAngAcc(const double time, Eigen::Quaterniond & quat_out,
				 					                                                   Eigen::Vector3d & ang_vel_out,
				 					                                                   Eigen::Vector3d & ang_acc_out){
	if (footstep_list_.size() == 0){
		return;
	}
	dcm_reference.get_ref_ori_ang_vel_acc(time, quat_out, ang_vel_out, ang_acc_out);
}

void DCMWalkingReferenceTrajectoryModule::getMPCRefComPosandVel(const double time, Eigen::Vector3d & x_com_out, Eigen::Vector3d & x_com_vel_out){
	if (footstep_list_.size() == 0){
		return;
	}
	dcm_reference.get_ref_com(time, x_com_out);
	dcm_reference.get_ref_com_vel(time, x_com_vel_out);	
}

// Get the maximum normal force
double DCMWalkingReferenceTrajectoryModule::getMaxNormalForce(int index, double time){
	return std::static_pointer_cast<DCMWalkingReactionForceSchedule>(reaction_force_schedule_ptr)->getMaxNormalForce(index,time);
}

// helper function to identify which footstep is in swing
// if false. the foot is in not in swing for the time queried
bool DCMWalkingReferenceTrajectoryModule::whichFootstepIndexInSwing(const double time, int & footstep_index){
	if (footstep_list_.size() == 0){
		return false;
	}

    // Not in swing if walking has not started yet.
    double t_query = time - t_walk_start_;
    if (t_query < 0.0){
        return false;
    }

	// Get the exponential interpolator step index
	int exp_step_index = dcm_reference.which_step_index(t_query);

	// Attempt to get the swing start and end times.
	double t_swing_start, t_swing_end;
	if (dcm_reference.get_t_swing_start_end(exp_step_index, t_swing_start, t_swing_end)){	
		// Ensure that the queried time is within the swing time.
		if ((t_swing_start <= t_query) && (t_query <= t_swing_end)){
			// Get the corresponding footstep index
			if (dcm_reference.rvrp_index_to_footstep_index.count(exp_step_index) > 0){
				// Set the footstep index
				int tmp_index = dcm_reference.rvrp_index_to_footstep_index[exp_step_index];
				// Ensure that there are no early contacts
	            // Go through the contact indices for this side of the foot
	            for (int j = 0; j < side_to_contact_indices[ footstep_list_[tmp_index].robot_side ].size() ; j++){
	                // Check if there are early contacts 
	                if ( (early_contact_times_.count(j) > 0) && (t_query >= early_contact_times_[j]) &&
	                     (early_contact_times_[j] >= t_swing_start)){               
	                    return false; // Early contact so this foot is no longer in swing.
	                }
	            }
				footstep_index = tmp_index;
	            return true;
			}
		}
	}else{
		// The step index is not a swing VRP type, return false		
		return false;
	}

	// t_query was not within the swing start and end times or the rvrp_index_to_footstep_index was empty.
	return false;

}

void DCMWalkingReferenceTrajectoryModule::saveSolution(const std::string & file_name) {
    try {
        double t_start = dcm_reference.getInitialTime();
        double t_end = t_start + dcm_reference.get_total_trajectory_time();
        double t_step(0.01);
        int n_eval = std::floor((t_end - t_start) / t_step);

        YAML::Node cfg;

        // =====================================================================
        // Temporal Parameters
        // =====================================================================

        cfg["temporal_parameters"]["initial_time"] = t_start;
        cfg["temporal_parameters"]["final_time"] = t_end;
        cfg["temporal_parameters"]["time_step"] = t_step;
        cfg["temporal_parameters"]["t_ds"] = dcm_reference.t_ds;
        cfg["temporal_parameters"]["t_ss"] = dcm_reference.t_ss;
        cfg["temporal_parameters"]["t_transfer"] = dcm_reference.t_transfer;

        // =====================================================================
        // Contact Information
        // =====================================================================
        int n_rf(0); int n_lf(0);
        for (int i = 0; i < footstep_list_.size(); ++i) {
            if (footstep_list_[i].robot_side == DRACO_LEFT_FOOTSTEP) {n_lf += 1;}
            else {n_rf += 1;}
        }
        Eigen::MatrixXd rfoot_pos = Eigen::MatrixXd::Zero(n_rf, 3);
        Eigen::MatrixXd rfoot_quat = Eigen::MatrixXd::Zero(n_rf, 4);
        Eigen::MatrixXd lfoot_pos = Eigen::MatrixXd::Zero(n_lf, 3);
        Eigen::MatrixXd lfoot_quat = Eigen::MatrixXd::Zero(n_lf, 4);
        int rf_id(0); int lf_id(0);
        for (int i = 0; i < footstep_list_.size(); ++i) {
            if (footstep_list_[i].robot_side == DRACO_RIGHT_FOOTSTEP) {
                for (int j = 0; j < 3; ++j) {rfoot_pos(rf_id, j) = footstep_list_[i].position(j);}
                rfoot_quat(rf_id, 0) = footstep_list_[i].orientation.w();
                rfoot_quat(rf_id, 1) = footstep_list_[i].orientation.x();
                rfoot_quat(rf_id, 2) = footstep_list_[i].orientation.y();
                rfoot_quat(rf_id, 3) = footstep_list_[i].orientation.z();
                rf_id += 1;
            } else {
                for (int j = 0; j < 3; ++j) {lfoot_pos(lf_id, j) = footstep_list_[i].position(j);}
                lfoot_quat(lf_id, 0) = footstep_list_[i].orientation.w();
                lfoot_quat(lf_id, 1) = footstep_list_[i].orientation.x();
                lfoot_quat(lf_id, 2) = footstep_list_[i].orientation.y();
                lfoot_quat(lf_id, 3) = footstep_list_[i].orientation.z();
                lf_id += 1;
            }
        }

        cfg["contact"]["right_foot"]["pos"] = rfoot_pos;
        cfg["contact"]["right_foot"]["ori"] = rfoot_quat;
        cfg["contact"]["left_foot"]["pos"] = rfoot_pos;
        cfg["contact"]["left_foot"]["ori"] = rfoot_quat;

        // =====================================================================
        // Reference Trajectory
        // =====================================================================
        Eigen::MatrixXd dcm_pos_ref = Eigen::MatrixXd::Zero(n_eval, 3);
        Eigen::MatrixXd dcm_vel_ref = Eigen::MatrixXd::Zero(n_eval, 3);
        Eigen::MatrixXd com_pos_ref = Eigen::MatrixXd::Zero(n_eval, 3);
        Eigen::MatrixXd com_vel_ref = Eigen::MatrixXd::Zero(n_eval, 3);
        Eigen::MatrixXd vrp_ref = Eigen::MatrixXd::Zero(n_eval, 3);

        double t(t_start);
        Eigen::Vector3d v3;
        for (int i = 0; i < n_eval; ++i) {
            dcm_reference.get_ref_dcm(t, v3);
            for (int j = 0; j < 3; ++j) {dcm_pos_ref(i, j) = v3(j);}
            dcm_reference.get_ref_dcm_vel(t, v3);
            for (int j = 0; j < 3; ++j) {dcm_vel_ref(i, j) = v3(j);}
            dcm_reference.get_ref_dcm_vel(t, v3);
            for (int j = 0; j < 3; ++j) {dcm_vel_ref(i, j) = v3(j);}
            dcm_reference.get_ref_com(t, v3);
            for (int j = 0; j < 3; ++j) {com_pos_ref(i, j) = v3(j);}
            dcm_reference.get_ref_com_vel(t, v3);
            for (int j = 0; j < 3; ++j) {com_vel_ref(i, j) = v3(j);}
            dcm_reference.get_ref_r_vrp(t, v3);
            for (int j = 0; j < 3; ++j) {vrp_ref(i, j) = v3(j);}
            t += t_step;
        }

        cfg["reference"]["dcm_pos"] = dcm_pos_ref;
        cfg["reference"]["dcm_vel"] = dcm_vel_ref;
        cfg["reference"]["com"] = com_pos_ref;
        cfg["reference"]["com_vel"] = com_vel_ref;
        cfg["reference"]["vrp"] = vrp_ref;

        std::string full_path = THIS_COM + std::string("ExperimentData/") +
            file_name + std::string(".yaml");
        std::ofstream file_out(full_path);
        file_out << cfg;

    }catch(YAML::ParserException& e) {
        std::cout << e.what() << std::endl;
    }
}
