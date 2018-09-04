#include <iostream>
#include "InvKinPlanner.hpp"
#include <memory>
#include <Configuration.h>
#include "Utilities.hpp"
#include <vector>

int main(int argc, char *argv[])
{
    std::shared_ptr<InvKinPlannerParameter> mParam =
        std::make_shared<InvKinPlannerParameter>();
    mParam->paramSetFromYaml(THIS_COM"Config/Draco/OneStepPlanning/CENTROID_PLANNER_RESULT.yaml");

    std::cout << "Done" << std::endl;
    return 0;
}
