#ifndef WALKING_REFERENCE_TRAJECTORY_MODULE_H
#define WALKING_REFERENCE_TRAJECTORY_MODULE_H

// Given:
//	  - starting time
//	  - starting starting configuration:
//			- starting com
//			- starting ori
//			- starting foot config
//    - sequence of footsteps to take,
//
// This class provides information about:
//	   State schedule s(t): (left leg single swing, right leg single swing, double support)	
//	   MPC CoM and orientation reference: x_com(t), x_ori(t)
//     Reaction force schedule: f(t)
//
// There is also an interface to provide early contact times.
// 

#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/MPC/ReactionForceSchedule.hpp>
#include <PnC/DracoPnC/PredictionModule/WalkingReactionForceSchedule.hpp>
#include <Eigen/Dense>

#include <stdio.h>
#include <vector>
#include <map>
#include <memory>


// states
#define DRACO_STATE_DS 0 // double support state
#define DRACO_STATE_LLS 1 // left leg swing state
#define DRACO_STATE_RLS 2 // right leg swing state

#define QUERY_BEFORE_TRAJECTORY -1 // if the query occurs before the trajectories
#define QUERY_AFTER_TRAJECTORY -2  // if the query occurs after the trajectories

class WalkingReactionForceSchedule;

class WalkingReferenceTrajectoryModule{
public:
	// Initialize by assigning the contact indices to a robot side.
	WalkingReferenceTrajectoryModule(const std::vector<int> & index_to_side_in);
	virtual ~WalkingReferenceTrajectoryModule();

	friend class WalkingReactionForceSchedule;

	virtual void setStartingConfiguration(const Eigen::Vector3d x_com_start_in,
								  const Eigen::Quaterniond x_ori_start_in,
								  const DracoFootstep & left_foot_start_in, 
								  const DracoFootstep & right_foot_start_in);
	// function which assigns a contact index to a robot side.
	virtual void setContactIndexToSide(const std::vector<int> & index_to_side_in);

	virtual void setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in);

	std::shared_ptr<ReactionForceSchedule> reaction_force_schedule_ptr;
	std::shared_ptr<WalkingReactionForceSchedule> walking_rfs_ptr;

	// gets the references 
	virtual int getState(const double time);
	virtual void getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out);
	virtual double getMaxNormalForce(int index, double time);

	// If true, populates the new footstep landing location
	// If false, the MPC should use the current location of the foot
	virtual bool getFutureMPCFootstep(double time, DracoFootstep & footstep_landing_location);

	// set that a particular contact was hit early
	// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
	// time: time of early contact
	virtual void setEarlyFootContact(const int index, const double time);

	// set that a particular foot was hit early. automatically handles the contact updates
	// robot_side DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
	// time: time of early contact
	virtual void setEarlyFootSideContact(const int robot_side, const double time);

	// helper function to identify which footstep is in swing
	// if false. the foot is in not in swing for the time queried
	virtual bool whichFootstepIndexInSwing(const double time, int & footstep_index);


protected:
	std::vector<int> index_to_side_;
	std::map<int, std::vector<int> > side_to_contact_indices;

	double t_walk_start_ = 0.0;

	Eigen::Vector3d x_com_start_;
	Eigen::Quaterniond x_ori_start_;
	DracoFootstep left_foot_start_;
	DracoFootstep right_foot_start_;

	// List of footsteps
	std::vector<DracoFootstep> footstep_list_; // list of footsteps
	std::map<int, double> early_contact_times_;
};

#endif