#include <fstream>
#include <numeric> // std::accumulate
#include <vector>

#include <configuration.h>

#include <towr_plus/locomotion_solution.h>
#include <towr_plus/models/endeffector_mappings.h>

LocomotionSolution::LocomotionSolution(const std::string &name,
                                       const YAML::Node &node) {
  name_ = name;
  int num_leg(2);
  ee_phase_durations_.resize(num_leg);
  n_ee_motion_nodes_.resize(num_leg);
  n_ee_motion_lin_vars_.resize(num_leg);
  n_ee_motion_ang_vars_.resize(num_leg);
  ee_motion_lin_nodes_.resize(num_leg);
  ee_motion_ang_nodes_.resize(num_leg);
  n_ee_wrench_nodes_.resize(num_leg);
  n_ee_wrench_vars_.resize(num_leg);
  ee_wrench_lin_nodes_.resize(num_leg);
  ee_wrench_ang_nodes_.resize(num_leg);
  ee_schedules_.resize(2);

  one_hot_ee_motion_lin_.resize(2);
  one_hot_ee_motion_ang_.resize(2);
  one_hot_ee_wrench_lin_.resize(2);
  one_hot_ee_wrench_ang_.resize(2);
  one_hot_ee_contact_schedule_.resize(2);

  Eigen::VectorXd tmp_vec;
  bool tmp_bool;
  try {
    ReadParameter(node, "duration_base_polynomial", duration_base_polynomial_);
    ReadParameter(node, "force_polynomials_per_stance_phase",
                  force_polynomials_per_stance_phase_);
    ReadParameter(node, "ee_polynomials_per_swing_phase",
                  ee_polynomials_per_swing_phase_);
    ReadParameter(node, "b_optimize_timings", b_optimize_timings_);
    assert(ee_polynomials_per_swing_phase_ == 2); // Assume this is always 2
    for (auto ee : {L, R}) {
      ReadParameter(node["ee_phase_durations"], std::to_string(ee), tmp_vec);
      for (int i = 0; i < tmp_vec.size(); ++i)
        ee_phase_durations_.at(ee).push_back(tmp_vec(i));
    }

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

LocomotionSolution::~LocomotionSolution() {}

void LocomotionSolution::print_info() {
  std::cout << "Locomotion Solution for " << name_ << std::endl;
}

void LocomotionSolution::print_solution(double dt) {

  using namespace std;
  cout.precision(2);
  cout << fixed;
  cout << "\n====================\nSolution "
          "trajectory:\n====================\n";

  double t = 0.0;
  while (t <= spline_holder_.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << spline_holder_.base_linear_->GetPoint(t).p().transpose() << "\t[m]"
         << endl;

    cout << "Base Euler roll, pitch, yaw:   \t";
    Eigen::Vector3d rad = spline_holder_.base_angular_->GetPoint(t).p();
    cout << (rad).transpose() << "\t[rad]" << endl;

    cout << "Left Foot position x,y,z:   \t";
    cout << spline_holder_.ee_motion_linear_.at(L)->GetPoint(t).p().transpose()
         << "\t[m]" << endl;

    cout << "Left Foot angular x,y,z:   \t";
    cout << spline_holder_.ee_motion_angular_.at(L)->GetPoint(t).p().transpose()
         << "\t[rad]" << endl;

    cout << "Right Foot position x,y,z:   \t";
    cout << spline_holder_.ee_motion_linear_.at(R)->GetPoint(t).p().transpose()
         << "\t[m]" << endl;

    cout << "Right Foot angular x,y,z:   \t";
    cout << spline_holder_.ee_motion_angular_.at(R)->GetPoint(t).v().transpose()
         << "\t[rad]" << endl;

    cout << "Left Foot Contact force x,y,z:   \t";
    cout << spline_holder_.ee_wrench_linear_.at(L)->GetPoint(t).p().transpose()
         << "\t[N]" << endl;

    cout << "Left Foot Contact trq x,y,z:   \t";
    cout << spline_holder_.ee_wrench_angular_.at(L)->GetPoint(t).v().transpose()
         << "\t[Nm]" << endl;

    cout << "Right Foot Contact force x,y,z:   \t";
    cout << spline_holder_.ee_wrench_linear_.at(R)->GetPoint(t).p().transpose()
         << "\t[N]" << endl;

    cout << "Right Foot Contact trq x,y,z:   \t";
    cout << spline_holder_.ee_wrench_angular_.at(R)->GetPoint(t).v().transpose()
         << "\t[Nm]" << endl;

    bool contact = spline_holder_.phase_durations_.at(L)->IsContactPhase(t);
    std::string foot_in_contact = contact ? "yes" : "no";
    cout << "Left Foot in contact:   \t" + foot_in_contact << endl;

    contact = spline_holder_.phase_durations_.at(R)->IsContactPhase(t);
    foot_in_contact = contact ? "yes" : "no";
    cout << "Right Foot in contact:   \t" + foot_in_contact << endl;

    cout << endl;

    t += 0.05;
  }
}

void LocomotionSolution::from_one_hot_vector(
    const Eigen::VectorXd &one_hot_vec) {
  one_hot_vector_ = one_hot_vec;

  parsing_idx_ = 0;
  _set_base_nodes();
  _set_ee_motion_nodes();
  _set_ee_wrench_nodes();
  _set_ee_schedule_variables();
  _set_splines();
}

void LocomotionSolution::_set_splines() {
  // Base Lin
  std::shared_ptr<NodesVariablesAll> base_lin =
      std::make_shared<NodesVariablesAll>(n_base_nodes_, k3D,
                                          id::base_lin_nodes);
  base_lin->SetVariables(one_hot_base_lin_);

  // Base Ang
  std::shared_ptr<NodesVariablesAll> base_ang =
      std::make_shared<NodesVariablesAll>(n_base_nodes_, k3D,
                                          id::base_ang_nodes);
  base_ang->SetVariables(one_hot_base_ang_);

  // EE Motion & Wrench & Contact Schedule
  std::vector<std::shared_ptr<NodesVariablesPhaseBased>> ee_motion_lin(2);
  std::vector<std::shared_ptr<NodesVariablesPhaseBased>> ee_motion_ang(2);
  std::vector<std::shared_ptr<NodesVariablesPhaseBased>> ee_wrench_lin(2);
  std::vector<std::shared_ptr<NodesVariablesPhaseBased>> ee_wrench_ang(2);
  std::vector<std::shared_ptr<PhaseDurations>> ee_phase_dur(2);
  for (auto ee : {L, R}) {
    ee_motion_lin.at(ee) = std::make_shared<NodesVariablesEEMotion>(
        ee_phase_durations_.at(ee).size(), true, id::EEMotionLinNodes(ee),
        ee_polynomials_per_swing_phase_, true);
    ee_motion_lin.at(ee)->SetVariables(one_hot_ee_motion_lin_.at(ee));

    ee_motion_ang.at(ee) = std::make_shared<NodesVariablesEEMotion>(
        ee_phase_durations_.at(ee).size(), true, id::EEMotionLinNodes(ee),
        ee_polynomials_per_swing_phase_, false);
    ee_motion_ang.at(ee)->SetVariables(one_hot_ee_motion_ang_.at(ee));

    ee_wrench_lin.at(ee) = std::make_shared<NodesVariablesEEForce>(
        ee_phase_durations_.at(ee).size(), true, id::EEWrenchLinNodes(ee),
        force_polynomials_per_stance_phase_);
    ee_wrench_lin.at(ee)->SetVariables(one_hot_ee_wrench_lin_.at(ee));

    ee_wrench_ang.at(ee) = std::make_shared<NodesVariablesEEForce>(
        ee_phase_durations_.at(ee).size(), true, id::EEWrenchLinNodes(ee),
        force_polynomials_per_stance_phase_);
    ee_wrench_ang.at(ee)->SetVariables(one_hot_ee_wrench_ang_.at(ee));

    ee_phase_dur.at(ee) = std::make_shared<PhaseDurations>(
        ee, ee_phase_durations_.at(ee), true, 0.,
        0.); // bound_phase_duration isn't important here
    ee_phase_dur.at(ee)->SetVariables(one_hot_ee_contact_schedule_.at(ee));
  }

  // Construct Splineholder
  spline_holder_ =
      SplineHolder(base_lin, base_ang, _get_base_poly_duration(), ee_motion_lin,
                   ee_motion_ang, ee_wrench_lin, ee_wrench_ang, ee_phase_dur,
                   b_optimize_timings_);
}

void LocomotionSolution::_set_base_nodes() {

  n_base_nodes_ = _get_base_poly_duration().size() + 1;
  n_base_vars_ = n_base_nodes_ * 6;

  one_hot_base_lin_ = one_hot_vector_.segment(parsing_idx_, n_base_vars_);
  parsing_idx_ += n_base_vars_;
  one_hot_base_ang_ = one_hot_vector_.segment(parsing_idx_, n_base_vars_);
  parsing_idx_ += n_base_vars_;

  base_lin_nodes_ = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      one_hot_base_lin_.data(), n_base_nodes_, 6);
  base_ang_nodes_ = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      one_hot_base_ang_.data(), n_base_nodes_, 6);
}

void LocomotionSolution::_set_ee_motion_nodes() {

  // Linear Motion
  // Assume ee phase starts and ends with contact
  for (auto ee : {L, R}) {
    n_ee_motion_nodes_.at(ee) =
        2 + (ee_polynomials_per_swing_phase_ + 1) *
                ((ee_phase_durations_.at(ee).size() - 2) + 1) / 2;
    n_ee_motion_lin_vars_.at(ee) =
        3 * (ee_phase_durations_.at(ee).size() + 1) / 2 +
        5 * (ee_phase_durations_.at(ee).size() - 1) / 2;
    ee_motion_lin_nodes_.at(ee) =
        Eigen::MatrixXd::Zero(n_ee_motion_nodes_.at(ee), 6);

    one_hot_ee_motion_lin_.at(ee) =
        one_hot_vector_.segment(parsing_idx_, n_ee_motion_lin_vars_.at(ee));
    parsing_idx_ += n_ee_motion_lin_vars_.at(ee);

    int node_idx(0);
    int variable_idx(0);
    for (int ph = 0; ph < ee_phase_durations_.at(ee).size(); ++ph) {
      if (ph % 2 == 0) {
        // Contact Phase: Use 3 variables (X, Y, Z) to fill 2 nodes
        // printf("filling node: %i, %i, with variable %i, %i, %i\n", node_idx,
        // node_idx + 1, variable_idx, variable_idx + 1, variable_idx + 2);
        for (auto dim : {0, 1, 2}) {
          ee_motion_lin_nodes_.at(ee)(node_idx, dim) =
              one_hot_ee_motion_lin_.at(ee)(variable_idx + dim);
          ee_motion_lin_nodes_.at(ee)(node_idx + 1, dim) =
              one_hot_ee_motion_lin_.at(ee)(variable_idx + dim);
        }
        node_idx += 2;
        variable_idx += 3;
      } else {
        // Swing Phase: Use 5 variables (X, DX, Y, DY, Z, DZ) to fill 1 node
        // printf("filling node: %i, with variable %i, %i, %i, %i, %i\n",
        // node_idx, variable_idx, variable_idx + 1, variable_idx + 2,
        // variable_idx + 3, variable_idx + 4);
        for (auto dim : {0, 1, 2, 3, 4, 5}) {
          if (dim == 0) {
            ee_motion_lin_nodes_.at(ee)(node_idx, 0) =
                one_hot_ee_motion_lin_.at(ee)(variable_idx + dim);
          } else if (dim == 1) {
            ee_motion_lin_nodes_.at(ee)(node_idx, 3) =
                one_hot_ee_motion_lin_.at(ee)(variable_idx + dim);
          } else if (dim == 2) {
            ee_motion_lin_nodes_.at(ee)(node_idx, 1) =
                one_hot_ee_motion_lin_.at(ee)(variable_idx + dim);
          } else if (dim == 3) {
            ee_motion_lin_nodes_.at(ee)(node_idx, 4) =
                one_hot_ee_motion_lin_.at(ee)(variable_idx + dim);
          } else if (dim == 4) {
            ee_motion_lin_nodes_.at(ee)(node_idx, 2) =
                one_hot_ee_motion_lin_.at(ee)(variable_idx + dim);
          } else if (dim == 5) {
            ee_motion_lin_nodes_.at(ee)(node_idx, 5) = 0.;
          } else {
            assert(false);
          }
        }
        node_idx += 1;
        variable_idx += 5;
      }
    }
  }

  // Angular Motion
  // Assume ee phase starts and ends with contact
  for (auto ee : {L, R}) {
    n_ee_motion_nodes_.at(ee) =
        2 + (ee_polynomials_per_swing_phase_ + 1) *
                ((ee_phase_durations_.at(ee).size() - 2) + 1) / 2;
    n_ee_motion_ang_vars_.at(ee) =
        3 * (ee_phase_durations_.at(ee).size() + 1) / 2 +
        6 * (ee_phase_durations_.at(ee).size() - 1) / 2;
    ee_motion_ang_nodes_.at(ee) =
        Eigen::MatrixXd::Zero(n_ee_motion_nodes_.at(ee), 6);

    one_hot_ee_motion_ang_.at(ee) =
        one_hot_vector_.segment(parsing_idx_, n_ee_motion_ang_vars_.at(ee));
    parsing_idx_ += n_ee_motion_ang_vars_.at(ee);

    int node_idx(0);
    int variable_idx(0);
    for (int ph = 0; ph < ee_phase_durations_.at(ee).size(); ++ph) {
      if (ph % 2 == 0) {
        // Contact Phase: Use 3 variables (X, Y, Z) to fill 2 nodes
        // printf("filling node: %i, %i, with variable %i, %i, %i\n", node_idx,
        // node_idx + 1, variable_idx, variable_idx + 1, variable_idx + 2);
        for (auto dim : {0, 1, 2}) {
          ee_motion_ang_nodes_.at(ee)(node_idx, dim) =
              one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
          ee_motion_ang_nodes_.at(ee)(node_idx + 1, dim) =
              one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
        }
        node_idx += 2;
        variable_idx += 3;
      } else {
        // Swing Phase: Use 5 variables (X, DX, Y, DY, Z, DZ) to fill 1 node
        // printf("filling node: %i, with variable %i, %i, %i, %i, %i\n",
        // node_idx, variable_idx, variable_idx + 1, variable_idx + 2,
        // variable_idx + 3, variable_idx + 4);
        for (auto dim : {0, 1, 2, 3, 4, 5}) {
          if (dim == 0) {
            ee_motion_ang_nodes_.at(ee)(node_idx, 0) =
                one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
          } else if (dim == 1) {
            ee_motion_ang_nodes_.at(ee)(node_idx, 3) =
                one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
          } else if (dim == 2) {
            ee_motion_ang_nodes_.at(ee)(node_idx, 1) =
                one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
          } else if (dim == 3) {
            ee_motion_ang_nodes_.at(ee)(node_idx, 4) =
                one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
          } else if (dim == 4) {
            ee_motion_ang_nodes_.at(ee)(node_idx, 2) =
                one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
          } else if (dim == 5) {
            ee_motion_ang_nodes_.at(ee)(node_idx, 5) =
                one_hot_ee_motion_ang_.at(ee)(variable_idx + dim);
          } else {
            assert(false);
          }
        }
        node_idx += 1;
        variable_idx += 6;
      }
    }
  }
}

void LocomotionSolution::_set_ee_wrench_nodes() {
  // Linear Wrench
  // Assume ee phase starts and ends with contact
  for (auto ee : {L, R}) {
    n_ee_wrench_nodes_.at(ee) = (force_polynomials_per_stance_phase_ + 1) *
                                (ee_phase_durations_.at(ee).size() + 1) / 2;
    n_ee_wrench_vars_.at(ee) =
        6 * (2 * force_polynomials_per_stance_phase_ +
             (((ee_phase_durations_.at(ee).size() + 1) / 2) - 2) *
                 (force_polynomials_per_stance_phase_ - 1));
    ee_wrench_lin_nodes_.at(ee) =
        Eigen::MatrixXd::Zero(n_ee_wrench_nodes_.at(ee), 6);

    one_hot_ee_wrench_lin_.at(ee) =
        one_hot_vector_.segment(parsing_idx_, n_ee_wrench_vars_.at(ee));
    parsing_idx_ += n_ee_wrench_vars_.at(ee);

    int node_idx(0);
    int variable_idx(0);
    for (int ph = 0; ph < ee_phase_durations_.at(ee).size(); ++ph) {
      if (ph % 2 == 0) {
        // Contact Phase
        if (ee_phase_durations_.at(ee).size() == 1) {
          // Handle the case of single phase
          ee_wrench_lin_nodes_.at(ee) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_lin_.at(ee).data(),
                  force_polynomials_per_stance_phase_ + 1, 6);
        } else if (ph == 0) {
          // Initial Contact Phase
          ee_wrench_lin_nodes_.at(ee).block(
              node_idx, 0, force_polynomials_per_stance_phase_, 6) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_lin_.at(ee)
                      .segment(variable_idx,
                               force_polynomials_per_stance_phase_ * 6)
                      .data(),
                  force_polynomials_per_stance_phase_, 6);

          node_idx += (force_polynomials_per_stance_phase_ + 1);
          variable_idx += force_polynomials_per_stance_phase_ * 6;

        } else if (ph == ee_phase_durations_.at(ee).size() - 1) {
          // Final Contact Phase
          ee_wrench_lin_nodes_.at(ee).block(
              node_idx + 1, 0, force_polynomials_per_stance_phase_, 6) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_lin_.at(ee)
                      .segment(variable_idx,
                               force_polynomials_per_stance_phase_ * 6)
                      .data(),
                  force_polynomials_per_stance_phase_, 6);

        } else {
          ee_wrench_lin_nodes_.at(ee).block(
              node_idx + 1, 0, force_polynomials_per_stance_phase_ - 1, 6) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_lin_.at(ee)
                      .segment(variable_idx,
                               (force_polynomials_per_stance_phase_ - 1) * 6)
                      .data(),
                  force_polynomials_per_stance_phase_ - 1, 6);
          node_idx += (force_polynomials_per_stance_phase_ + 1);
          variable_idx += (force_polynomials_per_stance_phase_ - 1) * 6;
        }
      } else {
        // Swing Phase: Do Nothing
      }
    }
    // Rearrange to make X, Y, Z, DX, DY, DZ
    ee_wrench_lin_nodes_.at(ee) =
        _transpose(ee_wrench_lin_nodes_.at(ee), {0, 2, 4, 1, 3, 5}, "col");
  }

  // Angular Wrench
  for (auto ee : {L, R}) {
    // n_ee_wrench_nodes and n_ee_wrench_vars are same as the ones for the
    // linear wrench.
    ee_wrench_ang_nodes_.at(ee) =
        Eigen::MatrixXd::Zero(n_ee_wrench_nodes_.at(ee), 6);

    one_hot_ee_wrench_ang_.at(ee) =
        one_hot_vector_.segment(parsing_idx_, n_ee_wrench_vars_.at(ee));
    parsing_idx_ += n_ee_wrench_vars_.at(ee);

    int node_idx(0);
    int variable_idx(0);
    for (int ph = 0; ph < ee_phase_durations_.at(ee).size(); ++ph) {
      if (ph % 2 == 0) {
        // Contact Phase
        if (ee_phase_durations_.at(ee).size() == 1) {
          // Handle the case of single phase
          ee_wrench_ang_nodes_.at(ee) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_ang_.at(ee).data(),
                  force_polynomials_per_stance_phase_ + 1, 6);
        } else if (ph == 0) {
          // Initial Contact Phase
          ee_wrench_ang_nodes_.at(ee).block(
              node_idx, 0, force_polynomials_per_stance_phase_, 6) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_ang_.at(ee)
                      .segment(variable_idx,
                               force_polynomials_per_stance_phase_ * 6)
                      .data(),
                  force_polynomials_per_stance_phase_, 6);

          node_idx += (force_polynomials_per_stance_phase_ + 1);
          variable_idx += force_polynomials_per_stance_phase_ * 6;

        } else if (ph == ee_phase_durations_.at(ee).size() - 1) {
          // Final Contact Phase
          ee_wrench_ang_nodes_.at(ee).block(
              node_idx + 1, 0, force_polynomials_per_stance_phase_, 6) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_ang_.at(ee)
                      .segment(variable_idx,
                               force_polynomials_per_stance_phase_ * 6)
                      .data(),
                  force_polynomials_per_stance_phase_, 6);

        } else {
          // Intermediate Contact Phase
          ee_wrench_ang_nodes_.at(ee).block(
              node_idx + 1, 0, force_polynomials_per_stance_phase_ - 1, 6) =
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
                  one_hot_ee_wrench_ang_.at(ee)
                      .segment(variable_idx,
                               (force_polynomials_per_stance_phase_ - 1) * 6)
                      .data(),
                  force_polynomials_per_stance_phase_ - 1, 6);
          node_idx += (force_polynomials_per_stance_phase_ + 1);
          variable_idx += (force_polynomials_per_stance_phase_ - 1) * 6;
        }
      } else {
        // Swing Phase: Do Nothing
      }
    }
    // Rearrange to make X, Y, Z, DX, DY, DZ
    ee_wrench_ang_nodes_.at(ee) =
        _transpose(ee_wrench_ang_nodes_.at(ee), {0, 2, 4, 1, 3, 5}, "col");
  }
}

void LocomotionSolution::_set_ee_schedule_variables() {
  if (b_optimize_timings_) {
    for (auto ee : {L, R}) {
      ee_schedules_.at(ee).clear();
      one_hot_ee_contact_schedule_.at(ee) = one_hot_vector_.segment(
          parsing_idx_, ee_phase_durations_.at(ee).size() - 1);
      parsing_idx_ += (ee_phase_durations_.at(ee).size() - 1);
      double sum(0.);
      for (int i = 0; i < ee_phase_durations_.at(ee).size(); ++i) {
        if (i == ee_phase_durations_.at(ee).size() - 1) {
          ee_schedules_.at(ee).push_back(_get_total_time() - sum);
        } else {
          ee_schedules_.at(ee).push_back(
              one_hot_ee_contact_schedule_.at(ee)(i));
          sum += one_hot_ee_contact_schedule_.at(ee)(i);
        }
      }
    }
  } else {
    for (auto ee : {L, R}) {
      one_hot_ee_contact_schedule_.at(ee) =
          Eigen::VectorXd::Zero(ee_phase_durations_.at(ee).size() - 1);
      for (int i = 0; i < one_hot_ee_contact_schedule_.at(ee).size(); ++i) {
        one_hot_ee_contact_schedule_.at(ee)(i) = ee_phase_durations_.at(ee)[i];
      }
    }
    ee_schedules_ = ee_phase_durations_;
  }
}

std::vector<double> LocomotionSolution::_get_base_poly_duration() {

  std::vector<double> base_spline_timings_;
  double dt = duration_base_polynomial_;
  double t_left = _get_total_time();

  double eps = 1e-10; // since repeated subtraction causes inaccuracies
  while (t_left > eps) {
    double duration = t_left > dt ? dt : t_left;
    base_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return base_spline_timings_;
}

double LocomotionSolution::_get_total_time() {
  std::vector<double> T_feet;

  for (const auto &v : ee_phase_durations_)
    T_feet.push_back(std::accumulate(v.begin(), v.end(), 0.0));

  // safety check that all feet durations sum to same value
  double T =
      T_feet.empty() ? 0.0 : T_feet.front(); // take first foot as reference
  for (double Tf : T_feet)
    assert(fabs(Tf - T) < 1e-6);

  return T;
}

Eigen::MatrixXd LocomotionSolution::_transpose(Eigen::MatrixXd mat,
                                               std::vector<int> order,
                                               std::string col_or_row) {
  int n_row = mat.rows();
  int n_col = mat.cols();
  Eigen::MatrixXd ret(n_row, n_col);
  if (col_or_row == "col") {
    assert(n_col == order.size());
    for (int i = 0; i < n_col; ++i) {
      ret.col(i) = mat.col(order[i]);
    }
  } else if (col_or_row == "row") {
    assert(n_row == order.size());
    for (int i = 0; i < n_row; ++i) {
      ret.row(i) = mat.row(order[i]);
    }
  } else {
    assert(false);
  }
  return ret;
}

void LocomotionSolution::to_yaml(double dt) {
  try {
    int n = std::floor(_get_total_time() / dt);
    Eigen::VectorXd time = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd base_lin = Eigen::MatrixXd::Zero(n, 6);
    Eigen::MatrixXd base_ang = Eigen::MatrixXd::Zero(n, 6);
    std::vector<Eigen::MatrixXd> ee_motion_lin(2);
    std::vector<Eigen::MatrixXd> ee_motion_ang(2);
    std::vector<Eigen::MatrixXd> ee_wrench_lin(2);
    std::vector<Eigen::MatrixXd> ee_wrench_ang(2);
    for (auto ee : {L, R}) {
      ee_motion_lin.at(ee) = Eigen::MatrixXd::Zero(n, 6);
      ee_motion_ang.at(ee) = Eigen::MatrixXd::Zero(n, 6);
      ee_wrench_lin.at(ee) = Eigen::MatrixXd::Zero(n, 6);
      ee_wrench_ang.at(ee) = Eigen::MatrixXd::Zero(n, 6);
    }

    for (int i = 0; i < n; ++i) {
      double t = i * dt;
      time(i) = t;
      base_lin.block(i, 0, 1, 3) =
          spline_holder_.base_linear_->GetPoint(t).p().transpose();
      base_lin.block(i, 3, 1, 3) =
          spline_holder_.base_linear_->GetPoint(t).v().transpose();
      base_ang.block(i, 0, 1, 3) =
          spline_holder_.base_angular_->GetPoint(t).p().transpose();
      base_ang.block(i, 3, 1, 3) =
          spline_holder_.base_angular_->GetPoint(t).v().transpose();
      for (auto ee : {L, R}) {
        ee_motion_lin.at(ee).block(i, 0, 1, 3) =
            spline_holder_.ee_motion_linear_.at(ee)
                ->GetPoint(t)
                .p()
                .transpose();
        ee_motion_lin.at(ee).block(i, 3, 1, 3) =
            spline_holder_.ee_motion_linear_.at(ee)
                ->GetPoint(t)
                .v()
                .transpose();
        ee_motion_ang.at(ee).block(i, 0, 1, 3) =
            spline_holder_.ee_motion_angular_.at(ee)
                ->GetPoint(t)
                .p()
                .transpose();
        ee_motion_ang.at(ee).block(i, 3, 1, 3) =
            spline_holder_.ee_motion_angular_.at(ee)
                ->GetPoint(t)
                .v()
                .transpose();
        ee_wrench_lin.at(ee).block(i, 0, 1, 3) =
            spline_holder_.ee_wrench_linear_.at(ee)
                ->GetPoint(t)
                .p()
                .transpose();
        ee_wrench_lin.at(ee).block(i, 3, 1, 3) =
            spline_holder_.ee_wrench_linear_.at(ee)
                ->GetPoint(t)
                .v()
                .transpose();
        ee_wrench_ang.at(ee).block(i, 0, 1, 3) =
            spline_holder_.ee_wrench_angular_.at(ee)
                ->GetPoint(t)
                .p()
                .transpose();
        ee_wrench_ang.at(ee).block(i, 3, 1, 3) =
            spline_holder_.ee_wrench_angular_.at(ee)
                ->GetPoint(t)
                .v()
                .transpose();
      }
    }

    YAML::Node data;

    data["trajectory"]["time"] = time;
    data["trajectory"]["base_lin"] = base_lin;
    data["trajectory"]["base_ang"] = base_ang;
    for (auto ee : {L, R}) {
      data["trajectory"]["ee_motion_lin"][std::to_string(ee)] =
          ee_motion_lin.at(ee);
      data["trajectory"]["ee_motion_ang"][std::to_string(ee)] =
          ee_motion_ang.at(ee);
      data["trajectory"]["ee_wrench_lin"][std::to_string(ee)] =
          ee_wrench_lin.at(ee);
      data["trajectory"]["ee_wrench_ang"][std::to_string(ee)] =
          ee_wrench_ang.at(ee);
    }

    Eigen::VectorXd tmp_vec;
    tmp_vec = Eigen::VectorXd::Zero(
        spline_holder_.base_linear_->GetPolyDurations().size());
    for (int i = 0; i < spline_holder_.base_linear_->GetPolyDurations().size();
         ++i) {
      tmp_vec(i) = spline_holder_.base_linear_->GetPolyDurations()[i];
    }
    data["node"]["base_lin"]["value"] = base_lin_nodes_;
    data["node"]["base_lin"]["duration"] = tmp_vec;
    data["node"]["base_ang"]["value"] = base_ang_nodes_;
    data["node"]["base_ang"]["duration"] = tmp_vec;
    for (auto ee : {L, R}) {
      data["node"]["ee_motion_lin"][std::to_string(ee)]["value"] =
          ee_motion_lin_nodes_.at(ee);
      data["node"]["ee_motion_ang"][std::to_string(ee)]["value"] =
          ee_motion_ang_nodes_.at(ee);
      tmp_vec = Eigen::VectorXd::Zero(
          spline_holder_.ee_motion_linear_.at(ee)->GetPolyDurations().size());
      for (int i = 0;
           i <
           spline_holder_.ee_motion_linear_.at(ee)->GetPolyDurations().size();
           ++i) {
        tmp_vec(i) =
            spline_holder_.ee_motion_linear_.at(ee)->GetPolyDurations()[i];
      }
      data["node"]["ee_motion_lin"][std::to_string(ee)]["duration"] = tmp_vec;
      data["node"]["ee_motion_ang"][std::to_string(ee)]["duration"] = tmp_vec;

      data["node"]["ee_wrench_lin"][std::to_string(ee)]["value"] =
          ee_wrench_lin_nodes_.at(ee);
      data["node"]["ee_wrench_ang"][std::to_string(ee)]["value"] =
          ee_wrench_ang_nodes_.at(ee);
      tmp_vec = Eigen::VectorXd::Zero(
          spline_holder_.ee_wrench_linear_.at(ee)->GetPolyDurations().size());
      for (int i = 0;
           i <
           spline_holder_.ee_wrench_linear_.at(ee)->GetPolyDurations().size();
           ++i) {
        tmp_vec(i) =
            spline_holder_.ee_wrench_linear_.at(ee)->GetPolyDurations()[i];
      }
      data["node"]["ee_wrench_lin"][std::to_string(ee)]["duration"] = tmp_vec;
      data["node"]["ee_wrench_ang"][std::to_string(ee)]["duration"] = tmp_vec;

      tmp_vec = Eigen::VectorXd::Zero(ee_schedules_.at(ee).size());
      for (int i = 0; i < ee_schedules_.at(ee).size(); ++i)
        tmp_vec(i) = ee_schedules_.at(ee)[i];
      data["contact_schedule"][std::to_string(ee)] = tmp_vec;
    }

    data["parameter"]["force_polynomials_per_stance_phase"] =
        force_polynomials_per_stance_phase_;
    data["parameter"]["ee_polynomials_per_swing_phase"] =
        ee_polynomials_per_swing_phase_;

    std::string file_path =
        THIS_COM + std::string("data/") + name_ + std::string(".yaml");
    std::ofstream file_out(file_path);
    file_out << data;

    std::cout << "Locomotion Solution " << name_ << " is saved" << std::endl;

  } catch (std::runtime_error &e) {
    std::cout << e.what() << std::endl;
  }
}
