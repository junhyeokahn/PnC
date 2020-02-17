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

// returns an output vector with size equal to dimension. 
Eigen::VectorXd StateTrajectoryWithinHorizon::getVal(const int index, const double time){
    Eigen::VectorXd out(dimension); 
    double val;
    if (index == STATE_TRAJECTORY_WITHIN_HORIZON_POSITION){
        for(int i = 0; i < dimension; i++){
            x_cubic[i].getPos(time, val);
            out[i] = val;
        }
    }
    else if (index == STATE_TRAJECTORY_WITHIN_HORIZON_VELOCITY){
        for(int i = 0; i < dimension; i++){
            x_cubic[i].getVel(time, val);
            out[i] = val;
        }
    }else if (index == STATE_TRAJECTORY_WITHIN_HORIZON_ACCELERATION){
        for(int i = 0; i < dimension; i++){
            x_cubic[i].getAcc(time, val);
            out[i] = val;
        }
    }else{
        out.setZero();
    }
    return out;
}

Eigen::VectorXd StateTrajectoryWithinHorizon::getPos(const double time){
    return getVal(STATE_TRAJECTORY_WITHIN_HORIZON_POSITION, time);
}

Eigen::VectorXd StateTrajectoryWithinHorizon::getVel(const double time){
    return getVal(STATE_TRAJECTORY_WITHIN_HORIZON_VELOCITY, time);
}
Eigen::VectorXd StateTrajectoryWithinHorizon::getAcc(const double time){
    return getVal(STATE_TRAJECTORY_WITHIN_HORIZON_ACCELERATION, time);
}


MPCDesiredTrajectoryManager::MPCDesiredTrajectoryManager(const int state_size_in, const int horizon_in){
    state_size = state_size_in;
    dim = floor(state_size/2);
    setHorizon(horizon_in);
    // std::cout << "state_size = " << state_size << std::endl;
    // std::cout << "dim = " << dim << std::endl;
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
        x_piecewise_cubic.push_back(StateTrajectoryWithinHorizon(dim));       
    }
}


void MPCDesiredTrajectoryManager::setStateKnotPoints(const double dt, const double t_start_in,
                                                     const Eigen::VectorXd & X_start,
                                                     const Eigen::VectorXd & X_pred){
    // Initialize trajectory start time
    t_start =  t_start_in;
    // Initialize horizon time window
    double t_horizon_begin, t_horizon_end;

    // Initialize the boundary conditions
    Eigen::VectorXd init_boundary(2*dim);
    Eigen::VectorXd end_boundary(2*dim);

    for(int i = 0; i < horizon; i++){
        t_horizon_begin = t_start + i*dt;
        t_horizon_end = t_start + (i+1)*dt;

        // Get the boundary conditions. The first boundary condition is the current state of the robot
        if (i == 0){
            init_boundary.head(dim) = X_start.head(dim);
            init_boundary.tail(dim) = X_start.segment(dim, dim);
        }else{
            init_boundary.head(dim) = X_pred.segment((i-1)*state_size, dim);
            init_boundary.tail(dim) = X_pred.segment((i-1)*state_size + dim, dim);
        }
        end_boundary.head(dim) = X_pred.segment(i*state_size, dim);
        end_boundary.tail(dim) = X_pred.segment(i*state_size + dim, dim);

        // Set the boundary conditions on the cubic polynomial
        x_piecewise_cubic[i].setParams(init_boundary, end_boundary, t_horizon_begin, t_horizon_end);
    }


}
















