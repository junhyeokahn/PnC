#include "PnC/ConvexMPC/ConvexMPC.hpp"
#include "PnC/ConvexMPC/GaitScheduler.hpp"
#include <PnC/RobotSystem/RobotSystem.hpp>

constexpr double desired_body_height = 0.3;
constexpr double desired_vx = 0.0;
constexpr double desired_vy = 0.0;
constexpr double desired_theta_dot = 0.0;
constexpr double mass = 9.713 + 0.5*(0.696 + 1.013 + 0.166 + 0.06) * 4;
constexpr int num_legs = 4;
constexpr int _PLANNING_HORIZON_STEPS = 10;
constexpr double _PLANNING_TIMESTEP = 0.025;

int main(int argc, char** argv){

    Eigen::Vector3d com_pos_des;
    com_pos_des << 0., 0., 0.3;
    Eigen::VectorXd _MPC_WEIGHTS(13);
    _MPC_WEIGHTS << 5, 5, 0.2, 0, 0, 10, 0.5, 0.5, 0.2, 0.2, 0.2, 0.1, 0;
    Eigen::VectorXd body_inertia(9);
    body_inertia << 0.02, 0, 0, 0, 0.06, 0, 0, 0, 0.07;
    Eigen::VectorXd friction_coeffs(4);
    friction_coeffs << 0.45, 0.45, 0.45, 0.45;
    ConvexMPC* mpc_;
    RobotSystem* robot_;
    GaitScheduler* gait_;

    robot_ = new RobotSystem(6, THIS_COM "RobotModel/Robot/A1/a1_sim.urdf");
    gait_ = new GaitScheduler(robot_);
    mpc_ = new ConvexMPC(
        mass,
        body_inertia,
        num_legs,
        _PLANNING_HORIZON_STEPS,
        _PLANNING_TIMESTEP,
        _MPC_WEIGHTS);
}
