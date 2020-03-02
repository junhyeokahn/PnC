#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>

// Initialize by assigning the contact indices to a robot side.
DCMWalkingReferenceTrajectoryModule::DCMWalkingReferenceTrajectoryModule
	(const std::vector<int> & index_to_side_in): WalkingReferenceTrajectoryModule(index_to_side_in){


    // t_walk_start_ = 0.0;
    // Initialize object pointers
    // reaction_force_schedule_ptr.reset(new WalkingReactionForceSchedule(this));
    // walking_rfs_ptr = std::static_pointer_cast<WalkingReactionForceSchedule>(reaction_force_schedule_ptr);
    // setContactIndexToSide(index_to_side_in);
    std::cout << "[DCMWalkingReferenceTrajectoryModule] Constructed" << std::endl;

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
	dcm_reference.initialize_footsteps_rvrp(footstep_list_, left_foot_start_, right_foot_start_, x_com_start_);
}

void DCMWalkingReferenceTrajectoryModule::getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out){

}

// helper function to identify which footstep is in swing
// if false. the foot is in not in swing for the time queried
bool DCMWalkingReferenceTrajectoryModule::whichFootstepIndexInSwing(const double time, int & footstep_index){
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