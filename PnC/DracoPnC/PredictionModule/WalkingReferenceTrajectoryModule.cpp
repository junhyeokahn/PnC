#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>

WalkingReferenceTrajectoryModule::WalkingReferenceTrajectoryModule(const std::vector<int> & index_to_side_in){
    t_walk_start_ = 0.0;

    // Initialize object pointers
    reaction_force_schedule_ptr.reset(new WalkingReactionForceSchedule());
    walking_rfs_ptr = std::static_pointer_cast<WalkingReactionForceSchedule>(reaction_force_schedule_ptr);

    setContactIndexToSide(index_to_side_in);
    std::cout << "[WalkingReferenceTrajectoryModule] Constructed" << std::endl;
}

WalkingReferenceTrajectoryModule::~WalkingReferenceTrajectoryModule(){}

void WalkingReferenceTrajectoryModule::setContactIndexToSide(const std::vector<int> & index_to_side_in){
    index_to_side_ = index_to_side_in;
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
    walking_rfs_ptr->setFootsteps(t_walk_start_in, footstep_list_in);
}

// gets the references 

// Check if we are in double support or one of the legs are in swing.
int WalkingReferenceTrajectoryModule::getState(const double time){
    int foot_index;
    if (whichFootstepIndexInSwing(time, foot_index)){
        if (footstep_list_[foot_index].robot_side == DRACO_LEFT_FOOTSTEP){
            return DRACO_STATE_LLS; // Left leg swing
        }else if (footstep_list_[foot_index].robot_side == DRACO_RIGHT_FOOTSTEP){
            return DRACO_STATE_RLS; // Right leg swing
        }
    }
    return DRACO_STATE_DS;
}

void WalkingReferenceTrajectoryModule::getMPCRefCom(const double time, Eigen::Vector3d & x_com){

}
void WalkingReferenceTrajectoryModule::getMPCRefOri(const double time, Eigen::Quaterniond & x_ori){

}
double WalkingReferenceTrajectoryModule::getMaxNormalForce(int index, double time){

}

// set that a particular contact was hit early
// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
// time: time of early contact
void WalkingReferenceTrajectoryModule::setEarlyFootContact(const int index, const double time){
}


// helper function to identify which footstep is in swing
// if false. the foot is in not in swing for the time queried
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

            // add check if time is after an early contact for this footstep

            footstep_index = i;
            return true;
        }
    }

    // Query time happens after the trajectories.
    return false;
}






































