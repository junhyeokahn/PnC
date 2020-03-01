#ifndef DCM_WALKING_REFERENCE_TRAJECTORY_MODULE_H
#define DCM_WALKING_REFERENCE_TRAJECTORY_MODULE_H

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

#include <PnC/DracoPnC/PredictionModule/DCMWalkingReference.hpp>
#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>

class DCMWalkingReferenceTrajectoryModule : public WalkingReferenceTrajectoryModule {
public:
	// Initialize by assigning the contact indices to a robot side.
	DCMWalkingReferenceTrajectoryModule(const std::vector<int> & index_to_side_in);
	virtual ~DCMWalkingReferenceTrajectoryModule();

	void setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in);

	// gets the references 
	int getState(const double time);
	void getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out);
	double getMaxNormalForce(int index, double time);

	// If true, populates the new footstep landing location
	// If false, the MPC should use the current location of the foot
	bool getFutureMPCFootstep(double time, DracoFootstep & footstep_landing_location);

	// set that a particular contact was hit early
	// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
	// time: time of early contact
	void setEarlyFootContact(const int index, const double time);

	// set that a particular foot was hit early. automatically handles the contact updates
	// robot_side DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
	// time: time of early contact
	void setEarlyFootSideContact(const int robot_side, const double time);

	// helper function to identify which footstep is in swing
	// if false. the foot is in not in swing for the time queried
	bool whichFootstepIndexInSwing(const double time, int & footstep_index);

	DCMWalkingReference dcm_reference;

};

#endif