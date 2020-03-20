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
#include <PnC/DracoPnC/PredictionModule/DCMWalkingReactionForceSchedule.hpp>
#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>

class DCMWalkingReferenceTrajectoryModule : public WalkingReferenceTrajectoryModule {
public:
	// Initialize by assigning the contact indices to a robot side.
	DCMWalkingReferenceTrajectoryModule(const std::vector<int> & index_to_side_in);
	virtual ~DCMWalkingReferenceTrajectoryModule();

	friend class DCMWalkingReactionForceSchedule;

	void setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in);

	void initialize(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in,
											const Eigen::Vector3d  dcm_pos_start_in,
											const Eigen::Vector3d  dcm_vel_start_in,	
										    const Eigen::Quaterniond & ori_start_in,
										    const DracoFootstep & left_foot_start_in,
										    const DracoFootstep & right_foot_start_in);

	// Get the maximum normal force
	double getMaxNormalForce(int index, double time);

	// gets the references 
	void getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out);
	void getMPCRefComPosandVel(const double time, Eigen::Vector3d & x_com_out, Eigen::Vector3d & x_com_vel_out);
	void getMPCRefQuatAngVelAngAcc(const double time, Eigen::Quaterniond & quat_out,
                                                      Eigen::Vector3d & ang_vel_out,
                                                      Eigen::Vector3d & ang_acc_out);

	// helper function to identify which footstep is in swing
	// if false. the foot is in not in swing for the time queried
	bool whichFootstepIndexInSwing(const double time, int & footstep_index);

	DCMWalkingReference dcm_reference;

};

#endif