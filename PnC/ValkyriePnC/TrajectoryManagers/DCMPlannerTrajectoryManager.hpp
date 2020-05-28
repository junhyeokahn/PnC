#pragma once

#include <PnC/TrajectoryManagerBase.hpp>
#include <PnC/PlannerSet/DCMPlanner/DCMPlanner.hpp>
#include <PnC/PlannerSet/DCMPlanner/Footstep.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>

namespace DCM_TRANSFER_TYPES {
    constexpr int INITIAL = 0;
    constexpr int MIDSTEP = 1;
};  

class DCMPlannerTrajectoryManager : public TrajectoryManagerBase {
  public:
  	DCMPlannerTrajectoryManager(DCMPlanner* _dcm_planner, RobotSystem* _robot);
  	~DCMPlannerTrajectoryManager();	
    void paramInitialization(const YAML::Node& node);

    bool initialize(const double t_walk_start_in, const std::vector<Footstep> & footstep_list_in,
                    const double t_transfer_in, 
                    const Eigen::Quaterniond & ori_start_in,
                    const Eigen::Vector3d & dcm_pos_start_in, 
                    const Eigen::Vector3d & dcm_vel_start_in);

    // Updates the feet pose of the starting stance 
    void updateStartingStance();
    void incrementStepIndex();
    void resetStepIndex();

    // Updates the local footstep list (ie: footstep preview) for trajectory generation:
    void updatePreview(const int max_footsteps_to_preview, std::vector<Footstep> & footstep_list);

    // Footstep sequence primitives -----------------------------------------------------------
    // Creates footstep in place
    void populateStepInPlace(const int num_steps, const int robot_side_first, std::vector<Footstep> & footstep_list);

    // Populates the input footstep list with a predefined walking forward behavior
    void populateWalkForward(const int num_steps,
    						 const double nominal_step_forward_distance,
    						 const double nominal_step_width_distance,
    						 const double midstep_distance_multiplier,
    						 std::vector<Footstep> & footstep_list);

    // Populates the input footstep list with a predefined footstep list to turn left
    void populateTurnLeft(const double turn_angle,
    					  const double nominal_step_width_distance,
    					  std::vector<Footstep> & footstep_list);

    // Populates the input footstep list with a predefined footstep list to turn right
    void populateTurnRight(const double turn_angle,
    					  const double nominal_step_width_distance,
    					  std::vector<Footstep> & footstep_list);

   DCMPlanner* dcm_planner_;
   std::vector<Footstep> footstep_list_copy_;
   std::vector<Footstep> footstep_preview_list_;
   int current_footstep_index_; // keeps track of which footstep to take.

   // Initialization
   double t_walk_start_;
   Eigen::Quaterniond ori_start_;
   Footstep right_foot_start_;
   Footstep left_foot_start_;


   Footstep right_foot_stance_;
   Footstep left_foot_stance_;

  // Human readable parameters 
  double t_additional_init_transfer_;        // the additional transfer time to switch the stance leg in the beginning
  double t_contact_transition_;  // the transition time used to change reaction forces and stance leg
  double t_swing_;               // the foot swing time. 

  // polynomial interpolation time during contact transition: t_transfer + t_ds + (1-alpha*t_ds).
  // DCM walking parameters
  double nominal_com_height_; 
  double t_transfer_init_; // = t_additional_init_transfer_ ; // additional transfer time offset
  double t_transfer_mid_; // = (alpha_ds_-1.0)*t_ds;  // transfer time offset for midstep transfers
  double t_ds_; // = t_contact_transition_; // double support polynomial transfer time
  double t_ss_; // = t_swing_; // single support exponential interpolation  time
  double percentage_settle_;// 0.99;//0.999; // percent to converge at the end of the trajectory
  double alpha_ds_;// = 0.5; // value between 0.0 and 1.0 for double support DCM interpolation


  // // Getter values for the contact transition time.
  // double t_initial_transfer_time = t_additional_transfer_ + t_ds + (1-alpha_ds_)*t_ds; // the total initial transfer time before the foot swinng
  // double t_midstep_transfer = t_ds; // midstep transfer time before contact transition
  // double t_final_transfer = t_ds + settle_time;
  double getInitialContactTransferTime();
  double getMidStepContactTransferTime();
  double getFinalContactTransferTime();
  double getSwingTime();

protected:
    void convertTemporalParamsToDCMParams();
};