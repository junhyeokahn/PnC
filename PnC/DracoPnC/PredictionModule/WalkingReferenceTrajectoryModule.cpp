#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>

WalkingReferenceTrajectoryModule::WalkingReferenceTrajectoryModule(const std::vector<int> & index_to_side_in){
    t_walk_start_ = 0.0;

    // Initialize object pointers
    reaction_force_schedule_ptr.reset(new WalkingReactionForceSchedule(this));
    walking_rfs_ptr = std::static_pointer_cast<WalkingReactionForceSchedule>(reaction_force_schedule_ptr);

    setContactIndexToSide(index_to_side_in);
    std::cout << "[WalkingReferenceTrajectoryModule] Constructed" << std::endl;
}

WalkingReferenceTrajectoryModule::~WalkingReferenceTrajectoryModule(){}

void WalkingReferenceTrajectoryModule::setContactIndexToSide(const std::vector<int> & index_to_side_in){
    index_to_side_ = index_to_side_in;

    side_to_contact_indices.clear();
    side_to_contact_indices[DRACO_LEFT_FOOTSTEP] = {};
    side_to_contact_indices[DRACO_RIGHT_FOOTSTEP] = {};
    for(int i = 0; i < index_to_side_.size(); i++){
        if (index_to_side_[i] == DRACO_LEFT_FOOTSTEP){
            side_to_contact_indices[DRACO_LEFT_FOOTSTEP].push_back(i);     
        }else {
            side_to_contact_indices[DRACO_RIGHT_FOOTSTEP].push_back(i);                 
        }
    }

}

// set the starting configuration
void WalkingReferenceTrajectoryModule::setStartingConfiguration(const Eigen::Vector3d x_com_start_in,
                              const Eigen::Quaterniond x_ori_start_in,
                              const DracoFootstep & left_foot_start_in, 
                              const DracoFootstep & right_foot_start_in){
    x_com_start_ = x_com_start_in;
    x_ori_start_ = x_ori_start_in;
    left_foot_start_ = left_foot_start_in;
    right_foot_start_ = right_foot_start_in;
}

//set the footsteps and starting time to walk
void WalkingReferenceTrajectoryModule::setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in){
    t_walk_start_ = t_walk_start_in;
    // Clear internal data then copy footsteps
    footstep_list_.clear();
    early_contact_times_.clear();
    footstep_list_ = footstep_list_in;
    // set footsteps on the reaction force schedule
}

// gets the references 

// Check if we are in double support or one of the legs are in swing.
int WalkingReferenceTrajectoryModule::getState(const double time){
    int foot_index;
    // Check if we are in swing. 
    if (whichFootstepIndexInSwing(time, foot_index)){
        if (footstep_list_[foot_index].robot_side == DRACO_LEFT_FOOTSTEP){
            return DRACO_STATE_LLS; // Left leg swing
        }else if (footstep_list_[foot_index].robot_side == DRACO_RIGHT_FOOTSTEP){
            return DRACO_STATE_RLS; // Right leg swing
        }
    }
    return DRACO_STATE_DS;
}

void WalkingReferenceTrajectoryModule::getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out){
    // Set output to initial
    x_com_out = x_com_start_;
    x_ori_out = x_ori_start_;
    if (time < t_walk_start_){
        return;
    }

    // check the initial stance foot 
    DracoFootstep stance_foot = left_foot_start_;
    if (footstep_list_.size() > 0){
        if (footstep_list_[0].robot_side == DRACO_RIGHT_FOOTSTEP){
            stance_foot = left_foot_start_;
        }else  if (footstep_list_[0].robot_side == DRACO_LEFT_FOOTSTEP){
            stance_foot = right_foot_start_;
        }
    }else{
        return;
    }

    // Go through the footstep list and set the CoM to the be the midfoot position of the stance and landing foot
    // set orientation to be the the midfoot orientation
    DracoFootstep landing_foot;
    DracoFootstep midfoot;
    double t_footstep_start = t_walk_start_;

    for(int i = 0; i < footstep_list_.size(); i++){
        // only update if the time query is greater than the starting footstep time
        if (time >= t_footstep_start){
            // set new landing foot and compute the midfoot
            landing_foot = footstep_list_[i];
            midfoot.computeMidfeet(stance_foot, landing_foot, midfoot);
            // set com x,y to midfoot
            x_com_out.head(2) = midfoot.position.head(2);
            // adjust com z height
            x_com_out[2] += (landing_foot.position[2] - stance_foot.position[2]);  
            // set orientation reference to midfoot
            x_ori_out = midfoot.orientation;

            // change the stance foot
            stance_foot = landing_foot;     
            // update new footstep start and repeat
            t_footstep_start += (footstep_list_[i].double_contact_time + 
                                 footstep_list_[i].contact_transition_time +
                                 footstep_list_[i].swing_time +
                                 footstep_list_[i].contact_transition_time);
            continue;
        }else{
            return;
        }
    }

    return;
}


double WalkingReferenceTrajectoryModule::getMaxNormalForce(int index, double time){
    return walking_rfs_ptr->getMaxNormalForce(index,time);
}

// set that a particular contact was hit early
// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
// time: time of early contact
void WalkingReferenceTrajectoryModule::setEarlyFootContact(const int index, const double time){
    early_contact_times_[index] = time;
}

void WalkingReferenceTrajectoryModule::setEarlyFootSideContact(const int robot_side, const double time){     
    for(int i = 0; i < side_to_contact_indices[robot_side].size(); i++){
        early_contact_times_[side_to_contact_indices[robot_side][i]] = time;
    }
}





// helper function to identify which footstep is in swing
// if false. the foot is in not in swing for the time queried or there was an early contact
bool WalkingReferenceTrajectoryModule::whichFootstepIndexInSwing(const double time, int & footstep_index){
    if (time < t_walk_start_){
        return false;
    }

    // go through the footstep list and check if the query time is within the footstep swing trajectory
    double t_footstep_swing_start = t_walk_start_; 
    double t_footstep_swing_end = t_walk_start_; 
    for(int i = 0; i < footstep_list_.size(); i++){
        t_footstep_swing_start = t_footstep_swing_end + footstep_list_[i].double_contact_time
                                                      + footstep_list_[i].contact_transition_time;
        t_footstep_swing_end = t_footstep_swing_start + footstep_list_[i].swing_time;

        if ((t_footstep_swing_start <= time) && (time <= t_footstep_swing_end)){

            // Go through the contact indices for this side of the foot
            for (int j = 0; j < side_to_contact_indices[ footstep_list_[i].robot_side ].size() ; j++){
                // If there are early contacts 
                if ( (early_contact_times_.count(j) > 0) && (time >= early_contact_times_[j]) &&
                     (early_contact_times_[j] >= t_footstep_swing_start)){               
                    return false;
                }
            }

            footstep_index = i;
            return true;
        }
    }

    // Query time happens after the trajectories.
    return false;
}