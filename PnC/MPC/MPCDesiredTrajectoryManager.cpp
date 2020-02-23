#include <PnC/MPC/MPCDesiredTrajectoryManager.hpp>


StateTrajectoryWithinHorizon::StateTrajectoryWithinHorizon(const int dimension_in){
    dimension = dimension_in;
    for(int i = 0; i < dimension; i++){
        x_cubic.push_back(CubicFit_OneDimension());     
        x_linear.push_back(LinearFit_OneDimension());
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
        x_linear[i].setParams(init_boundary[i], end_boundary[i], 
                              time_start, time_end);
    }

}

// returns an output vector with size equal to dimension. 
Eigen::VectorXd StateTrajectoryWithinHorizon::getVal(const int index, const double time, const int fit_type){
    Eigen::VectorXd out(dimension); 
    double val;
    if (index == STATE_TRAJECTORY_WITHIN_HORIZON_POSITION){
        for(int i = 0; i < dimension; i++){
            if (fit_type == STATE_TRAJECTORY_WITHIN_HORIZON_CUBIC_FIT){
                x_cubic[i].getPos(time, val);
            }else if (fit_type == STATE_TRAJECTORY_WITHIN_HORIZON_LINEAR_FIT){
                x_linear[i].getPos(time, val);
            }
            out[i] = val;
        }
    }
    else if (index == STATE_TRAJECTORY_WITHIN_HORIZON_VELOCITY){
        for(int i = 0; i < dimension; i++){
            if (fit_type == STATE_TRAJECTORY_WITHIN_HORIZON_CUBIC_FIT){
                x_cubic[i].getVel(time, val);
            }else if (fit_type == STATE_TRAJECTORY_WITHIN_HORIZON_LINEAR_FIT){
                x_linear[i].getVel(time, val);
            }
            out[i] = val;
        }

    }else if (index == STATE_TRAJECTORY_WITHIN_HORIZON_ACCELERATION){
        for(int i = 0; i < dimension; i++){
            if (fit_type == STATE_TRAJECTORY_WITHIN_HORIZON_CUBIC_FIT){
                x_cubic[i].getAcc(time, val);
            }else if (fit_type == STATE_TRAJECTORY_WITHIN_HORIZON_LINEAR_FIT){
                x_linear[i].getAcc(time, val);
            }
            out[i] = val;
        }
    }else{
        out.setZero();
    }

    return out;
}

Eigen::VectorXd StateTrajectoryWithinHorizon::getPos(const double time, const int fit_type){
    return getVal(STATE_TRAJECTORY_WITHIN_HORIZON_POSITION, time, fit_type);
}

Eigen::VectorXd StateTrajectoryWithinHorizon::getVel(const double time, const int fit_type){
    return getVal(STATE_TRAJECTORY_WITHIN_HORIZON_VELOCITY, time, fit_type);
}
Eigen::VectorXd StateTrajectoryWithinHorizon::getAcc(const double time, const int fit_type){
    return getVal(STATE_TRAJECTORY_WITHIN_HORIZON_ACCELERATION, time, fit_type);
}



InputTrajectoryWithinHorizon::InputTrajectoryWithinHorizon(const int dimension_in){
    dimension = dimension_in;
    for(int i = 0; i < dimension; i++){  
        u_linear.push_back(LinearFit_OneDimension());
    }
}

InputTrajectoryWithinHorizon::~InputTrajectoryWithinHorizon(){
}

// init_boundary is the input u at the start time
// end_boundary is the input u at the end boundary
void InputTrajectoryWithinHorizon::setParams(const Eigen::VectorXd init_boundary, 
                                             const Eigen::VectorXd end_boundary,
                                             const double time_start, double const time_end){

    for(int i = 0; i < dimension; i++){
        u_linear[i].setParams(init_boundary[i], end_boundary[i], time_start, time_end);
    }

}

Eigen::VectorXd InputTrajectoryWithinHorizon::getVal(const double time){
    Eigen::VectorXd out(dimension); 
    double val;
    for(int i = 0; i < dimension; i++){
        u_linear[i].getPos(time, val);
        out[i] = val;
    }
    return out;
}    


MPCDesiredTrajectoryManager::MPCDesiredTrajectoryManager(const int input_size_in, const int state_size_in, const int horizon_in, const double dt_in){
    input_size = input_size_in;
    state_size = state_size_in;
    dim = floor(state_size/2);
    setHorizon(horizon_in);
    setDt(dt_in);
    t_start = 0.0;

    linear_interpolate_states  = false;

    X_start_internal = Eigen::VectorXd::Zero(state_size);
    std::cout << "[MPCDesiredTrajectoryManager] Constructed" << std::endl;  
}

MPCDesiredTrajectoryManager::MPCDesiredTrajectoryManager(const int state_size_in, const int horizon_in, const double dt_in){
    input_size = 0;
    state_size = state_size_in;
    dim = floor(state_size/2);
    setHorizon(horizon_in);
    setDt(dt_in);
    t_start = 0.0;

    linear_interpolate_states  = false;

    X_start_internal = Eigen::VectorXd::Zero(state_size);
    std::cout << "[MPCDesiredTrajectoryManager] Constructed" << std::endl;  
}

void MPCDesiredTrajectoryManager::setLinearInterpolate(bool val){
    linear_interpolate_states = val;
}
void MPCDesiredTrajectoryManager::setCubicInterpolate(bool val){
    linear_interpolate_states = !val;
}

MPCDesiredTrajectoryManager::~MPCDesiredTrajectoryManager(){
    std::cout << "[MPCDesiredTrajectoryManager] Destroyed" << std::endl;
}

void MPCDesiredTrajectoryManager::setHorizon(const int horizon_in){
    horizon = horizon_in;
    X_pred_internal = Eigen::VectorXd::Zero(horizon*state_size);
    U_sequence_internal = Eigen::VectorXd::Zero(horizon*input_size);

    // Clear the trajectory containers
    x_trajectory.clear();
    u_trajectory.clear();

    for(int i = 0; i < horizon; i++){
        x_trajectory.push_back(StateTrajectoryWithinHorizon(dim));       
        u_trajectory.push_back(InputTrajectoryWithinHorizon(input_size));
    }

}

void MPCDesiredTrajectoryManager::setDt(const double dt_in){
    dt_internal = dt_in;
}

void MPCDesiredTrajectoryManager::setStateKnotPoints(const double t_start_in,
                                                     const Eigen::VectorXd & X_start,
                                                     const Eigen::VectorXd & X_pred){
    // Initialize trajectory start time
    t_start =  t_start_in;
    // Initialize horizon time window
    double t_horizon_begin, t_horizon_end;

    // Initialize the boundary conditions
    Eigen::VectorXd init_boundary(2*dim);
    Eigen::VectorXd end_boundary(2*dim);

    // Locally copy X_start and X_pred
    X_start_internal = X_start;
    X_pred_internal = X_pred;

    for(int i = 0; i < horizon; i++){
        t_horizon_begin = t_start + i*dt_internal;
        t_horizon_end = t_start + (i+1)*dt_internal;

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
        x_trajectory[i].setParams(init_boundary, end_boundary, t_horizon_begin, t_horizon_end);
    }
}

void MPCDesiredTrajectoryManager::setInputKnotPoints(const double t_start_in,
                                                     const Eigen::VectorXd & U_sequence){

    // Initialize trajectory start time
    t_start =  t_start_in;
    // Initialize horizon time window
    double t_horizon_begin, t_horizon_end;

    // Initialize the boundary conditions
    Eigen::VectorXd init_boundary(input_size);
    Eigen::VectorXd end_boundary(input_size);

    // Locally copy U_sequence
    U_sequence_internal = U_sequence;

    // Set parameters for the linear fit
    for(int i = 0; i < horizon; i++){
        t_horizon_begin = t_start + i*dt_internal;
        t_horizon_end = t_start + (i+1)*dt_internal;

        init_boundary = U_sequence.segment(i*input_size, input_size);
        if (i == (horizon - 1)){
            // Zero order hold at the last input
            end_boundary = U_sequence.segment(i*input_size, input_size);            
        }else{
            end_boundary = U_sequence.segment((i+1)*input_size, input_size);            
        }
        // Set the boundary conditions on the linear interpolation
        u_trajectory[i].setParams(init_boundary, end_boundary, t_horizon_begin, t_horizon_end);
    }    


}

void MPCDesiredTrajectoryManager::setStateAndInputKnotPoints(const double t_start_in,
                                                             const Eigen::VectorXd & X_start,
                                                             const Eigen::VectorXd & X_pred,
                                                             const Eigen::VectorXd & U_sequence){
    setStateKnotPoints(t_start_in, X_start, X_pred);
    setInputKnotPoints(t_start_in, U_sequence);
}


int MPCDesiredTrajectoryManager::getHorizonIndex(const double time){
    // get index to use for the piecewise cubic polynomial
    int horizon_index = (time - t_start)/dt_internal;
    // clamp index within bounds
    if (horizon_index < 0){
        horizon_index = 0;
    }    
    else if (horizon_index >= x_trajectory.size()){
        horizon_index = x_trajectory.size() - 1;
    }
    return horizon_index;    
}

Eigen::VectorXd MPCDesiredTrajectoryManager::getPos(const double time){
    // return the position
    int index = getHorizonIndex(time);
    int fit_type = STATE_TRAJECTORY_WITHIN_HORIZON_CUBIC_FIT;

    if (linear_interpolate_states){
        fit_type = STATE_TRAJECTORY_WITHIN_HORIZON_LINEAR_FIT;
    }

    return x_trajectory[index].getPos(time, fit_type);
}
Eigen::VectorXd MPCDesiredTrajectoryManager::getVel(const double time){
    // return the velocity
    int index = getHorizonIndex(time);
    int fit_type = STATE_TRAJECTORY_WITHIN_HORIZON_CUBIC_FIT;
    if (linear_interpolate_states){
        fit_type = STATE_TRAJECTORY_WITHIN_HORIZON_LINEAR_FIT;
    }

    return x_trajectory[index].getVel(time, fit_type);
}
Eigen::VectorXd MPCDesiredTrajectoryManager::getAcc(const double time){
    // return the acceleration
    int index = getHorizonIndex(time);
    int fit_type = STATE_TRAJECTORY_WITHIN_HORIZON_CUBIC_FIT;
    if (linear_interpolate_states){
        fit_type = STATE_TRAJECTORY_WITHIN_HORIZON_LINEAR_FIT;
    }
    return x_trajectory[index].getAcc(time, fit_type);
}

void MPCDesiredTrajectoryManager::getState(const double time_in, Eigen::VectorXd & x_out){
    x_out = X_start_internal;
    x_out.head(dim) = getPos(time_in);
    x_out.segment(dim, dim) = getVel(time_in);
}

void MPCDesiredTrajectoryManager::getInput(const double time_in, Eigen::VectorXd & u_out){
    // return the input for the requested time
    int index = getHorizonIndex(time_in);
    u_out = u_trajectory[index].getVal(time_in);    
}

Eigen::VectorXd MPCDesiredTrajectoryManager::getXpredVector(){
    return X_pred_internal;
}

Eigen::VectorXd MPCDesiredTrajectoryManager::getUSequence(){
    return U_sequence_internal;
}

// Returns the input state vector
Eigen::VectorXd MPCDesiredTrajectoryManager::getXStartVector(){
    return X_start_internal;
}
// Returns the global start time of the trajectories
double MPCDesiredTrajectoryManager::getStartTime(){
    return t_start;
}

// Returns the knot points that are after this time. if time exceeds the horizon, returns the last knotpoint
Eigen::VectorXd MPCDesiredTrajectoryManager::getTruncatedXpredVector(const double time){
    int index = getHorizonIndex(time);
    return X_pred_internal.tail((horizon-index)*state_size);
}

// Returns a vector of knotpoints, equal to the horizon, spaced by dt_internal excluding the input time
Eigen::VectorXd MPCDesiredTrajectoryManager::getInterpolatedXpredVector(const double time){
    Eigen::VectorXd X_pred_interpolated = X_pred_internal;
    Eigen::VectorXd X_pred_knotpoint(state_size);
    double t_local = time;
    for(int i = 0; i < horizon; i++){
        t_local = time + (i+1)*dt_internal;
        getState(t_local, X_pred_knotpoint);
        X_pred_interpolated.segment(i*state_size, state_size) = X_pred_knotpoint;
    }
    return X_pred_interpolated;
}


namespace mpc_trajectory_manager_test{
    // example functions
    double cube_func(double a, double t){
        return a*std::pow(t,3);
    }

    double cube_func_deriv(double a, double t){
        return 3*a*std::pow(t,2);
    }

    // Function which tests this object
    void test_object(){
        int state_size = 13;
        int horizon = 40;
        double mpc_dt = 0.025;
        MPCDesiredTrajectoryManager trajectory_manager(state_size, horizon, mpc_dt);

        double t = 0.0;
        double t_total = 45*mpc_dt;
        double dt = 0.01;
        int n = static_cast<int>(t_total/dt);

        // Set the starting state of the system
        Eigen::VectorXd X_start(state_size); X_start.setZero();
        X_start[12] = -9.81;
        // Set prediction 
        Eigen::VectorXd X_pred(state_size*horizon); X_pred.setZero();
        for(int i = 0; i < horizon; i++){
            // RPY
            X_pred[i*state_size + 0] = cube_func(1,(i+1)*mpc_dt);
            X_pred[i*state_size + 1] = 2.0*cube_func(1,(i+1)*mpc_dt); 
            X_pred[i*state_size + 2] = 3.0*cube_func(1,(i+1)*mpc_dt); 

            // XYZ
            X_pred[i*state_size + 3] = 4.0*cube_func(1,(i+1)*mpc_dt); 
            X_pred[i*state_size + 4] = 5.0*cube_func(1,(i+1)*mpc_dt); 
            X_pred[i*state_size + 5] = 6.0*cube_func(1,(i+1)*mpc_dt);

            // RPY rates
            X_pred[i*state_size + 6] = 1.0*cube_func_deriv(1,(i+1)*mpc_dt); 
            X_pred[i*state_size + 7] = 2.0*cube_func_deriv(1,(i+1)*mpc_dt); 
            X_pred[i*state_size + 8] = 3.0*cube_func_deriv(1,(i+1)*mpc_dt);

            // xyz velocities
            X_pred[i*state_size + 9] = 4.0*cube_func_deriv(1,(i+1)*mpc_dt); 
            X_pred[i*state_size + 10] = 5.0*cube_func_deriv(1,(i+1)*mpc_dt); 
            X_pred[i*state_size + 11] = 6.0*cube_func_deriv(1,(i+1)*mpc_dt);
        }

        for(int i = 0; i < horizon; i++){
        std::cout << i << ": X_pred = " << X_pred.segment(i*state_size, state_size).transpose() << std::endl;       
        }


        // Perform fit
        double time_start = 0.0;
        std::cout << "Pre fit" << std::endl;
        trajectory_manager.setStateKnotPoints(time_start, X_start, X_pred); 
        // trajectory_manager.setLinearInterpolateFirstStatetoNext(true);
        std::cout << "Post fit" << std::endl;

        Eigen::VectorXd x_traj; 
        for(int i = 0; i < n+1; i++){
            t = i*dt;
            trajectory_manager.getState(t, x_traj);
            std::cout << "t:" << t << " : " << x_traj.transpose() << std::endl;
        }
    }
}