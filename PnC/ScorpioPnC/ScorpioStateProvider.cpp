#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <PnC/ScorpioPnC/ScorpioStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

ScorpioStateProvider* ScorpioStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static ScorpioStateProvider state_provider_(_robot);
    return &state_provider_;
}

ScorpioStateProvider::ScorpioStateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "Scorpio State Provider");

    is_moving = false;

    phase_copy = 0;
    robot_ = _robot;
    curr_time = 0.;

    q = Eigen::VectorXd::Zero(Scorpio::n_dof);
    qdot = Eigen::VectorXd::Zero(Scorpio::n_dof);
    jpos_ini = Eigen::VectorXd::Zero(Scorpio::n_dof);

}

void ScorpioStateProvider::saveCurrentData() {

}
