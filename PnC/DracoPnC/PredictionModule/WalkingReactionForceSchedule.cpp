#include <PnC/DracoPnC/PredictionModule/WalkingReactionForceSchedule.hpp>

WalkingReactionForceSchedule::WalkingReactionForceSchedule(WalkingReferenceTrajectoryModule* reference_traj_module_in): 
    ReactionForceSchedule(){
    reference_traj_module = reference_traj_module_in;
}

WalkingReactionForceSchedule::~WalkingReactionForceSchedule(){
}

void WalkingReactionForceSchedule::testFunction(){
    std::cout << "hello size of footstep list is " << reference_traj_module->footstep_list_.size() << std::endl;

}

// default is to return the max z force
double WalkingReactionForceSchedule::getMaxNormalForce(const int index, const double time){
    // if index is out of bounds or time requested is earlier than walking start time
    if  ( ((index < 0) || (index >= reference_traj_module->index_to_side_.size())) ||
          (time <= reference_traj_module->t_walk_start_) ){
        return default_max_z_force_;
    }

    // if the footstep list is empty, return the default max z force
    if (reference_traj_module->footstep_list_.size() == 0){
        return default_max_z_force_;
    }

    // Go through the footstep list and check where the query occurs
    double t_contact_transition_start = reference_traj_module->t_walk_start_; 
    double t_contact_transition_end = reference_traj_module->t_walk_start_;
    double t_swing_start, t_swing_end, t_early, delta_t, t_o;
    double Fz = default_max_z_force_;
    double Fz_out = default_max_z_force_;

    int footstep_index;
    for(int i = 0; i < reference_traj_module->footstep_list_.size(); i++){
        footstep_index = i;
        // First Contact Transition
        // Decrease Max Z force from default max to 0.0 
        delta_t = reference_traj_module->footstep_list_[i].contact_transition_time;
        t_contact_transition_start = t_contact_transition_end + 
                                              reference_traj_module->footstep_list_[i].double_contact_time;

        t_contact_transition_end = t_contact_transition_start + delta_t;


        // Check if time query is within the first contact transition
        if ((t_contact_transition_start <= time) && (time <= t_contact_transition_end)){
            // compute transition force
            t_o = t_contact_transition_start;
            Fz_out = Fz + (-Fz/delta_t)*(time-t_o);
            break;
        }

        // Swing Phase
        // If we are in the swing phase. Check for early contact. Otherwise, Fz should be 0.0
        t_swing_start = t_contact_transition_end; 
        t_swing_end = t_swing_start + reference_traj_module->footstep_list_[i].swing_time;
        if ((t_swing_start <= time) && (time <= t_swing_end)){
            // Check for early contact.
            if (reference_traj_module->early_contact_times_.count(index) > 0){
                t_early = reference_traj_module->early_contact_times_[index];
                if ( (t_early >= t_swing_start) && (time >= t_early) ) {
                    t_o = t_early;
                    // Compute transition force
                    Fz_out = clampMaxFz((Fz/delta_t)*(time-t_o));
                    break;
                }
            }
            Fz_out = 0.0;
            break;
        }


        // Second Contact Transition
        // Increase Max Z force from 0.0 to default_max_z_force_ 
        t_contact_transition_start = t_swing_end;
        t_contact_transition_end = t_contact_transition_start + delta_t;

        // check if time query is within this contact transition
        if ((t_contact_transition_start <= time) && (time <= t_contact_transition_end)){
            // Check for early contact
            // Adjust t_o if there is an early contact for this footstep
            if (reference_traj_module->early_contact_times_.count(index) > 0){
                t_early = reference_traj_module->early_contact_times_[index];
                if ( (t_early >= t_swing_start) && (time >= t_early) ) {
                    t_o = t_early;
                }
            }else{
                t_o = t_contact_transition_start;
            }

            // Compute transition force
            Fz_out = clampMaxFz( (Fz/delta_t)*(time-t_o) );
            break;
        }

    }

    // Assumption that only one foot contact at a time enters a transition state.
    // check if the contact index query matches the robot side. 
    // if it doesn't match, this foot is not in a contact transition phase. return default value
    if (reference_traj_module->index_to_side_[index] != reference_traj_module->footstep_list_[footstep_index].robot_side){
        return default_max_z_force_;
    }

    return Fz_out;
}


double WalkingReactionForceSchedule::clampMaxFz(double Fz_in){
    if (Fz_in >= default_max_z_force_){
        return default_max_z_force_;
    }else{
        return Fz_in;
    }
}

