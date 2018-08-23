#include <iostream>
#include "CentroidPlanner.hpp"
#include <memory>
#include "Configuration.h"
#include "Utilities.hpp"
#include <vector>

int main(int argc, char *argv[])
{
    std::shared_ptr<CentroidPlannerParameter> mParam =
        std::make_shared<CentroidPlannerParameter>();
    mParam->paramSetFromYaml(THIS_COM"Config/Draco/CENTROID_PLANNER.yaml");

    std::unique_ptr<CentroidPlanner> mPlanner =
        std::make_unique<CentroidPlanner>();

    mPlanner->updatePlanningParameter(mParam);
    double time(1.0);
    Eigen::VectorXd pos, vel, trq;
    mPlanner->getPlan(time, pos, vel, trq);
    std::cout << "Done" << std::endl;
    return 0;
}
