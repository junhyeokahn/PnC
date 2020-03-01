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
	void getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out);

	// helper function to identify which footstep is in swing
	// if false. the foot is in not in swing for the time queried
	bool whichFootstepIndexInSwing(const double time, int & footstep_index);

	DCMWalkingReference dcm_reference;

};

#endif