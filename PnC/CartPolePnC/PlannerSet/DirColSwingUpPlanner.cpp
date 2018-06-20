#include "CartPolePnC/PlannerSet/DirColSwingUpPlanner.hpp"
#include "Configuration.h"

DirColSwingUpPlanner::DirColSwingUpPlanner(
        std::shared_ptr<DirColSwingUpPlanningParameter> param_) :
    Planner( (std::shared_ptr<PlanningParameter>) param_) {
    mTree = std::make_unique< RigidBodyTree<double> >();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            THIS_COM"Simulator/SimulationModel/RobotModel/CartPole/CartPole.urdf",
            drake::multibody::joints::kFixed, mTree.get());
    mPlant = std::make_unique< drake::systems::RigidBodyPlant<double> >(
            std::move(mTree), 0.0);
    printf("[DirCol Swing Up Test] Constructed\n");
}

DirColSwingUpPlanner::~DirColSwingUpPlanner() {}

void DirColSwingUpPlanner::_doPlan() {

}

void DirColSwingUpPlanner::_evalTrajecotry( double time,
                                            Eigen::VectorXd & pos,
                                            Eigen::VectorXd & vel,
                                            Eigen::VectorXd acc,
                                            Eigen::VectorXd &eff ) {

}
