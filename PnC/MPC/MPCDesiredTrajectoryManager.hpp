#ifndef H_MPC_DESIRED_TRAJECTORY_MANAGER
#define H_MPC_DESIRED_TRAJECTORY_MANAGER


#include <Utils/Math/cubicfit_one_dim.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

// A container that holds the state trajectory within the time horizon bounded by time_start and time_end

#define STATE_TRAJECTORY_WITHIN_HORIZON_POSITION 0
#define STATE_TRAJECTORY_WITHIN_HORIZON_VELOCITY 1
#define STATE_TRAJECTORY_WITHIN_HORIZON_ACCELERATION 2

class StateTrajectoryWithinHorizon{
public:
    // for a given state [x, xdot], the size of dimension_in is equal to the dimension of x.
    StateTrajectoryWithinHorizon(const int dimension_in);
    ~StateTrajectoryWithinHorizon();

    // init_boundary is the state [x, xdot] at the start time
    // end_boundary is the state [x, xdot] at the end boundary
    void setParams(const Eigen::VectorXd init_boundary, 
                   const Eigen::VectorXd end_boundary,
                   const double time_start, double const time_end);

    Eigen::VectorXd getVal(const int index, const double time);    

    Eigen::VectorXd getPos(const double time);
    Eigen::VectorXd getVel(const double time);
    Eigen::VectorXd getAcc(const double time);

private:
    int dimension;
    // A vector of a polynomial cubic fit for the state within the horizon 
    std::vector<CubicFit_OneDimension> x_cubic;
};


class MPCDesiredTrajectoryManager{
public: 
    MPCDesiredTrajectoryManager(const int state_size_in, const int horizon_in, const double dt_in);
    ~MPCDesiredTrajectoryManager();

    // Accepts a concatenated state vector which lists the knot points of the state over the horizon
    // Assumes that the state has the form X = [x, \dot{x}]
    // For CMPC, the state has the form X = [x, \dot{x}, g] where g is the gravitational constant. 
    //    this class automatically handles the CMPC case due to the implementation.

    // Therefore, X_pred = [X_1, X_2, ..., X_horizon].

    // Warning. It's important that X_pred has the right dimension
    // double t_start_in, starting time of the reference trajectory

    // Eigen::VectorXd X_start = [x_start, \dot{x}_start]; // starting state of the system
    void setStateKnotPoints(const double t_start_in,
                            const Eigen::VectorXd & X_start,
                            const Eigen::VectorXd & X_pred); 

    // int horizon: number of time steps
    void setHorizon(const int horizon_in);
    // double dt: time interval between knot points. Usually equal to the MPC dt
    void setDt(const double dt_in);

    // outputs the state value at the specified time
    // x_out = [x_cubic(t), \dot{x}_cubic(t)]
    
    // time_in is clamped between (t_start and t_start + horizon*dt_internal)
    void getState(const double time_in, Eigen::VectorXd & x_out); 

    // helper function which gives the index to use for the piecewise cubic function
    int getHorizonIndex(const double time);

    Eigen::VectorXd getPos(const double time);
    Eigen::VectorXd getVel(const double time);
    Eigen::VectorXd getAcc(const double time);

private:
    double t_start; // global start time of the trajectories
    double dt_internal;

    int state_size;
    int dim;
    int horizon;

    // vector containing all the polynomial cubic fits from t_start to t_start + horizon*dt
    std::vector< StateTrajectoryWithinHorizon > x_piecewise_cubic;


};

namespace mpc_trajectory_manager_test{
    // Function which tests this object
    void test_object();
}

#endif