#pragma once

#include <PnC/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <Eigen/Dense>


class ReactionForceTrajectoryManager : public TrajectoryManagerBase {
public:
  ReactionForceTrajectoryManager(double _mpc_dt, int _mpc_horizon);
  ~ReactionForceTrajectoryManager() {};

  void updateSolution(double curr_time, Eigen::VectorXd _mpc_solution);

  Eigen::VectorXd getRFSolution(double curr_time);
protected:
  double sol_init_time;
  double mpc_dt;
  int mpc_horizon;

  Eigen::VectorXd full_rxn_force_vector;
  Eigen::VectorXd single_rxn_force_vector;
  std::vector<Eigen::VectorXd> vec_single_force_vectors;
}
