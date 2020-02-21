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
    // if index is out of bounds or time requested is earlier than 
    if  ( ((index < 0) || (index >= reference_traj_module->footstep_list_.size())) ||
          (time <= reference_traj_module->t_walk_start_) ){
        return default_max_z_force_;
    }

    // Go through the footstep list and check where the query occurs
    double t_footstep_contact_transition_start = reference_traj_module->t_walk_start_; 
    double t_footstep_contact_transition_end = reference_traj_module->t_walk_start_;
    double t_swing_end;

    double delta_t, t_o;
    double Fz = default_max_z_force_;
    double Fz_out = Fz;
    for(int i = 0; i < reference_traj_module->footstep_list_.size(); i++){
        // First Contact Transition
        // Decrease Max Z force from default max to 0.0 
        t_footstep_contact_transition_start = t_footstep_contact_transition_end + 
                                              reference_traj_module->footstep_list_[i].double_contact_time;

        t_footstep_contact_transition_end = t_footstep_contact_transition_start +
                                            reference_traj_module->footstep_list_[i].contact_transition_time;

        delta_t = reference_traj_module->footstep_list_[i].contact_transition_time;

        // Check if time query is within the first contact transition
        if ((t_footstep_contact_transition_start <= time) && (time <= t_footstep_contact_transition_end)){
            // compute transition force
            t_o = t_footstep_contact_transition_start;
            Fz_out = Fz + (-Fz/delta_t)*(time-t_o);
            return Fz_out;
        }

        // Swing Phase
        // If we are in the swing phase. Check for early contact. Otherwise, Fz should be 0.0
        t_swing_end = t_footstep_contact_transition_end + reference_traj_module->footstep_list_[i].swing_time;
        if ((t_footstep_contact_transition_end <= time) && (time <= t_swing_end)){
            // check for early contact. if index in early_contact && (time >= early_contact[index])
            // t_o = t_footstep_contact_transition_start;
            // Fz_out = (Fz/delta_t)*(time-t_o);
            // clamp maximum force out if the contact was detected early
            // if (Fz_out >= default_max_z_force_){
            //     return default_max_z_force_;
            // }
            // return Fz_out;
            return 0.0;
        }


        // Second Contact Transition
        // Increase Max Z force from 0.0 to default_max_z_force_ 
        t_footstep_contact_transition_start = t_swing_end;
        t_footstep_contact_transition_end = t_footstep_contact_transition_start +
                                            reference_traj_module->footstep_list_[i].contact_transition_time;

        // check if time query is within this contact transition
        if ((t_footstep_contact_transition_start <= time) && (time <= t_footstep_contact_transition_end)){
            // adjust t_o if there is an early contact for this footstep

            // compute transition force
            t_o = t_footstep_contact_transition_start;
            Fz_out = (Fz/delta_t)*(time-t_o);

            // clamp maximum force out if the contact was detected early
            if (Fz_out >= default_max_z_force_){
                return default_max_z_force_;
            }
            return Fz_out;
        }

    }

    // query time is not within the contact transition phase
    return default_max_z_force_;




}




