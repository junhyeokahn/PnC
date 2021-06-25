#pragma once

#include <PnC/WBC/Contact.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

class ReactionForceManager {
public:
  ReactionForceManager(Contact *_contact, double maximum_rf_z_max);
  ~ReactionForceManager();

  void InitializeRampToMin(double start_time, double dur);
  void InitializeRampToMax(double start_time, double dur);
  void UpdateRampToMin(double _curr_time);
  void UpdateRampToMax(double curr_time);

protected:
  Contact *contact_;
  double maximum_rf_z_max_;
  double minimum_rf_z_max_;
  double starting_rf_z_max_;
  double start_time_;
  double duration_;
};
