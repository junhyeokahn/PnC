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
    return default_max_z_force_;
}

double DCMWalkingReactionForceSchedule::clampMaxFz(double Fz_in){
    if (Fz_in >= default_max_z_force_){
        return default_max_z_force_;
    }else{
        return Fz_in;
    }
}

