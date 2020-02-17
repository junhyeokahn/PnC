#include <PnC/MPC/MPCDesiredTrajectoryManager.hpp>


StateTrajectoryWithinHorizon::StateTrajectoryWithinHorizon(const int dimension_in){
    dimension = dimension_in;
    for(int i = 0; i < dimension; i++){
        x_cubic.push_back(CubicFit_OneDimension());     
    }
    // std::cout << "[StateTrajectoryWithinHorizon] cubic fit for state size " << dimension << " constructed" << std::endl;

}
StateTrajectoryWithinHorizon::~StateTrajectoryWithinHorizon(){}


// init_boundary = [x(time_start), xdot(time_start)]
// end_boundary = [x(time_end), xdot(time_end)]
void StateTrajectoryWithinHorizon::setParams(const Eigen::VectorXd init_boundary, 
                                             const Eigen::VectorXd end_boundary,
                                             const double time_start, const double time_end){
    for(int i = 0; i < dimension; i++){
        x_cubic[i].setParams(Eigen::Vector2d(init_boundary[i], init_boundary[dimension + i]),
                             Eigen::Vector2d(end_boundary[i], end_boundary[dimension + i]), 
                             time_start, time_end);        
    }

}

Eigen::VectorXd getPos(const double time);
Eigen::VectorXd getVel(const double time);
Eigen::VectorXd getAcc(const double time);

MPCDesiredTrajectoryManager::MPCDesiredTrajectoryManager(const int state_size_in, const int horizon_in){
    state_size = state_size_in;
    state_size_to_interpolate = floor(state_size/2);
    setHorizon(horizon_in);
    // std::cout << "state_size = " << state_size << std::endl;
    // std::cout << "state_size_to_interpolate = " << state_size_to_interpolate << std::endl;
    std::cout << "[MPCDesiredTrajectoryManager] Constructed" << std::endl;  
}

MPCDesiredTrajectoryManager::~MPCDesiredTrajectoryManager(){
    std::cout << "[MPCDesiredTrajectoryManager] Destroyed" << std::endl;
}

void MPCDesiredTrajectoryManager::setHorizon(const int horizon_in){
    horizon = horizon_in;
    // Clear the piecewise cubic container
    x_piecewise_cubic.clear();
    for(int i = 0; i < horizon; i++){
        x_piecewise_cubic.push_back(StateTrajectoryWithinHorizon(state_size_to_interpolate));       
    }
}