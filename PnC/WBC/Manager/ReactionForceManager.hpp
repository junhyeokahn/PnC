#pragma once

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/Contact.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

class ReactionForceManager {
public:
  ReactionForceManager(Contact *_contact, double maximum_rf_z_max,
                       RobotSystem *_robot);
  ~ReactionForceManager();

  void initialize_ramp_to_min(double start_time, double dur);
  void initialize_ramp_to_max(double start_time, double dur);
  void update_ramp_to_min(double curr_time);
  void update_ramp_to_max(double curr_time);

protected:
  RobotSystem *robot_;
  Contact *contact_;
  double maximum_rf_z_max_;
  double minimum_rf_z_max_;
  double starting_rf_z_max_;
  double start_time_;
  double duration_;
};
