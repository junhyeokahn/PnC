#pragma once

#include "pnc/whole_body_controllers/contact.hpp"
#include "utils/util.hpp"

/// class TaskHierarchyManager
class ReactionForceManager {
public:
  /// \{ \name Constructor and Destructor
  ReactionForceManager(Contact *_contact, double maximum_rf_z_max);

  ~ReactionForceManager();
  /// \}

  /// Initialize reaction force upper bound transition to min
  void InitializeRampToMin(double start_time, double dur);

  /// Initialize reaction force upper bound transition to max
  void InitializeRampToMax(double start_time, double dur);

  /// Update reaction force upper bound
  void UpdateRampToMin(double _curr_time);

  /// Update reaction force upper bound
  void UpdateRampToMax(double curr_time);

protected:
  Contact *contact_;
  double maximum_rf_z_max_;
  double minimum_rf_z_max_;
  double starting_rf_z_max_;
  double start_time_;
  double duration_;
};
