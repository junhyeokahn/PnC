#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

ValkyrieStateProvider* ValkyrieStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static ValkyrieStateProvider state_provider_(_robot);
    return &state_provider_;
}

ValkyrieStateProvider::ValkyrieStateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "Valkyrie State Provider");

    num_step_copy = 0;
    phase_copy = 0;
    robot_ = _robot;
    stance_foot = ValkyrieBodyNode::leftFoot;
    curr_time = 0.;

    q = Eigen::VectorXd::Zero(Valkyrie::n_dof);
    qdot = Eigen::VectorXd::Zero(Valkyrie::n_dof);

    b_rfoot_contact = 0;
    b_lfoot_contact = 0;
}
