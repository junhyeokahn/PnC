
// Given:
//	  - starting time
//	  - starting starting configuration:
//			- starting com
//			- starting ori
//			- starting foot config
//    - sequence of footsteps to take,
//
// This class provides information about:
//     Contact schedule c(t): (binary value if contact is active or not)
//	   State schedule s(t): (left leg single support, right leg single support, double support)	
//	   MPC CoM and orientation reference: x_com(t), x_ori(t)
//     Reaction force schedule: f(t)
//	   Left and Right Foot location x_rf(t), x_lf(t)
//
// There is also an interface to provide early contact times.
// 

#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/DracoPnC/PredictionModule/WalkingReactionForceSchedule.hpp>
#include <Eigen/Dense>
#include <memory>

// states
#define DRACO_STATE_DS 0 // double support state
#define DRACO_STATE_LLS 1 // left leg swing state
#define DRACO_STATE_RLS 2 // right leg swing state

class WalkingReferenceTrajectoryModule{
public:
	WalkingReferenceTrajectoryModule();
	~WalkingReferenceTrajectoryModule();

	void setStartingConfiguration(const Eigen::Vector3d x_com_start_in,
								  const Eigen::Quaterniond x_ori_start_in,
								  const DracoFootstep & left_foot_start_in, 
								  const DracoFootstep & right_foot_start_in);
	void setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in);

	std::shared_ptr<ReactionForceSchedule> reaction_force_schedule_ptr;
	std::shared_ptr<WalkingReactionForceSchedule> walking_rfs_ptr;

	// gets the references 
	int getState(const double time);
	void getMPCRefCom(const double time, Eigen::Vector3d x_com);
	void getMPCRefOri(const double time, Eigen::Quaterniond x_ori);
	double getMaxNormalForce(int index, double time);

	// set that a particular contact was hit early
	// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
	// time: time of early contact
	void setEarlyFootContact(const int index, const double time);

private:
	double t_walk_start_ = 0.0;

	Eigen::Vector3d x_com_start_;
	Eigen::Quaterniond x_ori_start_;
	DracoFootstep left_foot_start_;
	DracoFootstep right_foot_start_;


	// List of footsteps
	std::vector<DracoFootstep> footstep_list_; // list of footsteps
	std::map<int, double> early_contact_times_;
};
