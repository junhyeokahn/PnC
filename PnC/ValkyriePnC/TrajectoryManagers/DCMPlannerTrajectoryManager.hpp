#pragma once

#include <PnC/TrajectoryManagerBase.hpp>
#include <PnC/PlannerSet/DCMPlanner/DCMPlanner.hpp>
#include <PnC/PlannerSet/DCMPlanner/Footstep.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>

class DCMPlannerTrajectoryManager : public TrajectoryManagerBase {
  public:
  	DCMPlannerTrajectoryManager(DCMPlanner* _dcm_planner, RobotSystem* _robot);
  	~DCMPlannerTrajectoryManager();	
    void paramInitialization(const YAML::Node& node);

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
   std::vector<Footstep> footstep_preview_list_;
   int current_footstep_index_; // keeps track of which footstep to take.

   Footstep right_foot_start_;
   Footstep left_foot_start_;

   double t_initial_transfer_;    // the initial transfer time
   double t_double_support_;	  // the time during double support
   double t_contact_transition_;  // the transition time used to change reaction forces
   								  // 	- defined as a ratio of t_double_support_
   double t_swing_; 			  // the foot swing time. 

};