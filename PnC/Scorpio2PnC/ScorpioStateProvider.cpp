#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/Scorpio2PnC/ScorpioDefinition.hpp>
#include <PnC/Scorpio2PnC/ScorpioStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

Scorpio2StateProvider* Scorpio2StateProvider::getStateProvider(
    RobotSystem* _robot) {
    static Scorpio2StateProvider state_provider_(_robot);
    return &state_provider_;
}

Scorpio2StateProvider::Scorpio2StateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "Scorpio2 State Provider");

    is_moving = false;

    //Gripper Status boolean variable
    is_closing = false;
    is_holding = false;
    is_opening = false;

    closing_opening_start_time = 0;

    phase_copy = 0;
    robot_ = _robot;
    curr_time = 0.;

    q = Eigen::VectorXd::Zero(Scorpio::n_dof);
    qdot = Eigen::VectorXd::Zero(Scorpio::n_dof);
    jpos_ini = Eigen::VectorXd::Zero(Scorpio::n_dof);

    

}

void Scorpio2StateProvider::saveCurrentData() {

}
