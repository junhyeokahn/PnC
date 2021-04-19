#include <PnC/TrajectoryManager/ReactionForceTrajectoryManager.hpp>


ReactionForceTrajectoryManager::ReactionForceTrajectoryManager(double _mpc_dt,
                                                               int _mpc_horizon,
                                                               RobotSystem* _robot)
            : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: Reaction Force Interpolation");
  full_rxn_force_vector = Eigen::VectorXd::Zero(120);
  single_rxn_force_vector = Eigen::VectorXd::Zero(12);
  mpc_dt = _mpc_dt;
  mpc_horizon = _mpc_horizon;
  vec_single_force_vectors.reserve(mpc_horizon);
}

void ReactionForceTrajectoryManager::updateSolution(
                                    double curr_time,
                                    Eigen::VectorXd _mpc_solution){
  sol_init_time = curr_time;
  full_rxn_force_vector = _mpc_solution;
  vec_single_force_vectors.clear();
  std::cout << "rxn force tm 1" << std::endl;
  std::cout << "full_rxn_force_vector.size() = " << full_rxn_force_vector.size() << std::endl;
  for(int i=0; i<mpc_horizon; ++i) {
    single_rxn_force_vector[0] = full_rxn_force_vector[12*i + 0];
    single_rxn_force_vector[1] = full_rxn_force_vector[12*i + 1];
    single_rxn_force_vector[2] = full_rxn_force_vector[12*i + 2];
    single_rxn_force_vector[3] = full_rxn_force_vector[12*i + 3];
    single_rxn_force_vector[4] = full_rxn_force_vector[12*i + 4];
    single_rxn_force_vector[5] = full_rxn_force_vector[12*i + 5];
    single_rxn_force_vector[6] = full_rxn_force_vector[12*i + 6];
    single_rxn_force_vector[7] = full_rxn_force_vector[12*i + 7];
    single_rxn_force_vector[8] = full_rxn_force_vector[12*i + 8];
    single_rxn_force_vector[9] = full_rxn_force_vector[12*i + 9];
    single_rxn_force_vector[10] = full_rxn_force_vector[12*i + 10];
    single_rxn_force_vector[11] = full_rxn_force_vector[12*i + 11];
    std::cout << "rxn force tm 2" << std::endl;
    vec_single_force_vectors.push_back(single_rxn_force_vector);
  }
  std::cout << "vec_single_force_vectors.size() = " << vec_single_force_vectors.size() << std::endl;
}

Eigen::VectorXd ReactionForceTrajectoryManager::getRFSolution(double curr_time) {
  Eigen::VectorXd vec1, vec2;
  vec1 = Eigen::VectorXd::Zero(12);
  vec2 = Eigen::VectorXd::Zero(12);
  double percent;
  if((curr_time - sol_init_time) < 0.025){
      vec1 = vec_single_force_vectors[0];
      vec2 = vec_single_force_vectors[1];
      percent = (curr_time - sol_init_time) / 0.025;
  }
  else if(((curr_time - sol_init_time) >= 0.025) && ((curr_time - sol_init_time) < 0.05)){
      vec1 = vec_single_force_vectors[1];
      vec2 = vec_single_force_vectors[2];
      percent = (curr_time - sol_init_time) / 0.025;
  }
  else if((curr_time - sol_init_time) >= 0.05){
      std::cout << "curr_time - sol_init_time larger than expected" << std::endl;
      exit(0);
  }
  return vec1*(1 - percent) + vec2*(percent);
}
