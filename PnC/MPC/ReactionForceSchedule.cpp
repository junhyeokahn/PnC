#include <PnC/MPC/ReactionForceSchedule.hpp>

ReactionForceSchedule::ReactionForceSchedule(){

}

ReactionForceSchedule::~ReactionForceSchedule(){

}

void ReactionForceSchedule::setDefaultMaxNormalForce(double default_max_z_force_in){
	default_max_z_force = default_max_z_force_in;
}

double ReactionForceSchedule::getMaxNormalForce(int index, double time){
	return default_max_z_force;
}

double ReactionForceSchedule::getDefaultMaxNormalForce(){
	return default_max_z_force;
}