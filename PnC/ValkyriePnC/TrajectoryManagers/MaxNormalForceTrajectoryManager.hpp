#pragma once

#include <Utils/Math/MathUtilities.hpp>
#include <PnC/TrajectoryManagerBase.hpp>
#include <PnC/WBC/BasicContactSpec.hpp>

// Object to manage the maximum values of reaction forces.

// Call in the following order
// 1) initializeRampToZero/Max in the initialization step with the current time and nominal ramp duration
// 2) computeRampToZero() in the update step with the current time
// 3) updateMaxNormalForce() in the update step after computing 

class MaxNormalForceTrajectoryManager : public TrajectoryManagerBase {
  public:
    MaxNormalForceTrajectoryManager(ContactSpec* _contact, RobotSystem* _robot);
    ~MaxNormalForceTrajectoryManager(); 
    void paramInitialization(const YAML::Node& node);

    ContactSpec* contact_;
    double nominal_max_normal_force_z_;

    // Initializes the ramp speed needed to bring the current max nominal force to zero or max.
    // - Stores the starting time and value of the current nominal force
    // ramp_down_speed = -nominal_max_normal_force_z_ / nominal_ramp_duration 
    // ramp_up_speed = nominal_max_normal_force_z_ / nominal_ramp_duration    
    void initializeRampToZero(const double start_time, const double nominal_ramp_duration);
    void initializeRampToMax(const double start_time, const double nominal_ramp_duration);

    double ramp_start_time_ ; // time at which ramping starts
    double starting_max_normal_force_z_; // starting normal force value

    // Updates the current max normal force given the current time and temporal parameters
    void computeRampToZero(const double current_time);
    void computeRampToMax(const double current_time);

    double current_max_normal_force_z_;  // current normal force value to update contact spec

    // computes the new normal force and updates the maximum force normal force
    void updateRampToZeroDesired(const double current_time);
    void updateRampToMaxDesired(const double current_time);

    // Update the maximum normal force 
    void updateMaxNormalForce();

  protected:
    double local_max_normal_force_z_;
    double ramp_up_speed_ = 1;
    double ramp_down_speed_ = 1;
};