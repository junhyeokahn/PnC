#include <PnC/DracoPnC/PredictionModule/DCMWalkingReactionForceSchedule.hpp>

DCMWalkingReactionForceSchedule::DCMWalkingReactionForceSchedule(DCMWalkingReferenceTrajectoryModule* reference_traj_module_in): 
    ReactionForceSchedule(){
    reference_traj_module = reference_traj_module_in;
}

DCMWalkingReactionForceSchedule::~DCMWalkingReactionForceSchedule(){
}

void DCMWalkingReactionForceSchedule::testFunction(){
    std::cout << "hello size of footstep list is " << reference_traj_module->footstep_list_.size() << std::endl;

}

// default is to return the max z force
double DCMWalkingReactionForceSchedule::getMaxNormalForce(const int index, const double time){
    // if index is out of bounds or time requested is earlier than walking start time
    if  ( ((index < 0) || (index >= reference_traj_module->index_to_side_.size())) ||
          (time <= reference_traj_module->t_walk_start_) ){
        return default_max_z_force_;
    }

    // if the footstep list is empty, return the default max z force
    if (reference_traj_module->footstep_list_.size() == 0){
        return default_max_z_force_;
    }

    // Set t_query time
    double t_query = time - reference_traj_module->t_walk_start_;
	// Get the exponential interpolator step index
	int exp_step_index = reference_traj_module->dcm_reference.which_step_index(t_query);

    double t_contact_transition_start, t_contact_transition_end;
    double t_swing_start, t_swing_end, t_early, delta_t, t_o;
    double Fz = default_max_z_force_;
    double Fz_out = default_max_z_force_;

    int footstep_index;

    // Ensure that the VRP is a swing step
	if (reference_traj_module->dcm_reference.get_t_swing_start_end(exp_step_index, t_swing_start, t_swing_end)){	
		// Get the corresponding footstep index
		if (reference_traj_module->dcm_reference.rvrp_index_to_footstep_index.count(exp_step_index) > 0){
			// Set the footstep index
			footstep_index = reference_traj_module->dcm_reference.rvrp_index_to_footstep_index[exp_step_index];

			// First Contact Transition
			// Decrease Max Z Force
			t_contact_transition_start = reference_traj_module->dcm_reference.get_t_step_start(exp_step_index);
			t_contact_transition_end = t_swing_start;
			delta_t = t_contact_transition_end - t_contact_transition_start;
			if ((t_contact_transition_start <= t_query) && (t_query <= t_contact_transition_end)){
				t_o = t_contact_transition_start;
	            Fz_out = Fz + (-Fz/delta_t)*(t_query-t_o);
			}

			// Swing Phase
			t_contact_transition_start = t_swing_end;
			t_contact_transition_end = reference_traj_module->dcm_reference.get_t_step_end(exp_step_index);
			delta_t = t_contact_transition_end - t_contact_transition_start;
			// Max Z force must be 0.0 or must be increasing due to early contact.
			if ((t_swing_start <= t_query) && (t_query <= t_swing_end)){
				// Check for early contact
				Fz_out = 0.0;

			}

			// Second Contact Transition
			// Increase Max Z Force
			if ((t_contact_transition_start <= t_query) && (t_query <= t_contact_transition_end)){
				// Check for early contact
             	t_o = t_contact_transition_start;

	            // Compute transition force
	            Fz_out = clampMaxFz( (Fz/delta_t)*(time-t_o) );
			}

		    // Assumption that only one foot contact at a time enters a transition state.
		    // check if the contact index query matches the robot side. 
		    // if it doesn't match, this foot is not in a contact transition phase. return default value
		    if (reference_traj_module->index_to_side_[index] != reference_traj_module->footstep_list_[footstep_index].robot_side){
		        return default_max_z_force_;
		    }
		    // Otherwise return the computed transition force
		   return Fz_out;
		}

	}else{
		// VRP is not in swing. Set default max z force 
		return default_max_z_force_;
	}


    return default_max_z_force_;
}

double DCMWalkingReactionForceSchedule::clampMaxFz(double Fz_in){
    if (Fz_in >= default_max_z_force_){
        return default_max_z_force_;
    }else{
        return Fz_in;
    }
}

