#pragma once

#include <PnC/TrajectoryManagerBase.hpp>
#include <PnC/PlannerSet/DCMPlanner/DCMPlanner.hpp>
#include <PnC/PlannerSet/DCMPlanner/Footstep.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>

#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>

namespace DCM_TRANSFER_TYPES {
    constexpr int INITIAL = 0;
    constexpr int MIDSTEP = 1;
};  

class DCMPlannerTrajectoryManager : public TrajectoryManagerBase {
  public:
  	DCMPlannerTrajectoryManager(DCMPlanner* _dcm_planner, RobotSystem* _robot);
  	~DCMPlannerTrajectoryManager();	
    void paramInitialization(const YAML::Node& node);

    bool initialize(const double t_walk_start_in,
                    const int transfer_type_in, 
                    const Eigen::Quaterniond & ori_start_in,
                    const Eigen::Vector3d & dcm_pos_start_in, 
                    const Eigen::Vector3d & dcm_vel_start_in);

    // Walking Primitives
    void walkInPlace();
    void walkForward();
    void walkBackward();
    void strafeLeft();
    void strafeRight();
    void turnLeft();
    void turnRight();
    void resetIndexAndClearFootsteps();
    void alternateLeg();

    // Updates the feet pose of the starting stance 
    void updateStartingStance();

    int getStepIndex(){
      return current_footstep_index_;
    }

    void incrementStepIndex();
    void resetStepIndex();

    // Updates the local footstep list (ie: footstep preview) for trajectory generation:
    void updatePreview(const int max_footsteps_to_preview);

    // Footstep sequence primitives -----------------------------------------------------------
    // Creates footstep in place
    void populateStepInPlace(const int num_steps, const int robot_side_first);

    // Populates the input footstep list with a predefined walking forward behavior
    void populateWalkForward(const int num_steps,
    						             const double forward_distance);

    // Rotate at the specified turn angle
    void populateRotateTurn(const double turn_radians_per_step, const int num_times);

    // Rotate at the specified turn angle
    void populateStrafe(const double strafe_distance, const int num_times);


   DCMPlanner* dcm_planner_;
   std::vector<Footstep> footstep_list_;
   std::vector<Footstep> footstep_preview_list_;
   int current_footstep_index_; // keeps track of which footstep to take.

   // Initialization
   double t_walk_start_;
   Eigen::Quaterniond ori_start_;
   Footstep right_foot_start_;
   Footstep left_foot_start_;

   Footstep right_foot_stance_;
   Footstep left_foot_stance_;
   Footstep mid_foot_stance_;

   // Nominal walking parameter primitives
  double nominal_footwidth_;
  double nominal_forward_step_;
  double nominal_backward_step_;
  double nominal_turn_radians_;
  double nominal_strafe_distance_;   

  int robot_side_first_;


  // Human readable parameters  DCM parameters
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

  double getNormalForceRampUpTime();
  double getNormalForceRampDownTime();

  // Returns false if footstep_list is empty or current_step_index_ is greater than the footstep list
  // populates the robot_side when true.
  bool nextStepRobotSide(int & robot_side);
  
  // checks whether or not there are emaining footsteps.
  bool noRemainingSteps();

  void setCoMandPelvisTasks(Task* _com_task, Task* _pelvis_ori_task_);
  void updateDCMTasksDesired(double current_time);
  Task* com_task_;
  Task* pelvis_ori_task_;

protected:
    void convertTemporalParamsToDCMParams();
};