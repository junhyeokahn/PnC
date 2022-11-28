#include "pnc/whole_body_controllers/managers/reaction_force_manager.hpp"

ReactionForceManager::ReactionForceManager(
    Contact *_contact, const double _maximum_rf_z_max,
    const bool b_use_smooth_interpolation) {
  util::PrettyConstructor(2, "ReactionForceManager");

  contact_ = _contact;
  maximum_rf_z_max_ = _maximum_rf_z_max;
  minimum_rf_z_max_ = 0.001;
  starting_rf_z_max_ = contact_->rf_z_max;
  start_time_ = 0.;
  duration_ = 0.;
  b_use_smooth_interpolation_ = b_use_smooth_interpolation;
}

ReactionForceManager::~ReactionForceManager() {}

void ReactionForceManager::InitializeRampToMin(double _start_time,
                                               double _dur) {
  start_time_ = _start_time;
  duration_ = _dur;
  starting_rf_z_max_ = contact_->rf_z_max;
}

void ReactionForceManager::InitializeRampToMax(double _start_time,
                                               double _dur) {
  start_time_ = _start_time;
  duration_ = _dur;
  starting_rf_z_max_ = contact_->rf_z_max;
}

void ReactionForceManager::UpdateRampToMin(double _curr_time) {
  double t = util::Clamp(_curr_time, start_time_, start_time_ + duration_);

  contact_->rf_z_max = b_use_smooth_interpolation_
                           ? (minimum_rf_z_max_ - starting_rf_z_max_) /
                                     std::pow(duration_, 2) *
                                     std::pow((t - start_time_), 2) +
                                 starting_rf_z_max_
                           : (minimum_rf_z_max_ - starting_rf_z_max_) /
                                     duration_ * (t - start_time_) +
                                 starting_rf_z_max_;
}

void ReactionForceManager::UpdateRampToMax(double _curr_time) {
  double t = util::Clamp(_curr_time, start_time_, start_time_ + duration_);
  contact_->rf_z_max = b_use_smooth_interpolation_
                           ? (maximum_rf_z_max_ - starting_rf_z_max_) /
                                     std::pow(duration_, 2) *
                                     std::pow((t - start_time_), 2) +
                                 starting_rf_z_max_
                           : (maximum_rf_z_max_ - starting_rf_z_max_) /
                                     duration_ * (t - start_time_) +
                                 starting_rf_z_max_;
}
