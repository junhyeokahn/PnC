#include <iostream>
#include "PnC/PlannerSet/InvKinPlanner/InvKinPlanner.hpp"
#include <memory>
#include <Configuration.h>
#include "Utils/Utilities.hpp"
#include <vector>

/*int main(int argc, char *argv[])*/
//{
    //std::shared_ptr<InvKinPlannerParameter> mParam =
        //std::make_shared<InvKinPlannerParameter>();
    //mParam->paramSetFromYaml(THIS_COM"Config/Draco/PLANNER/ONE_STEP_PLANNING/CENTROID_PLANNER_RESULT.yaml");
    //Eigen::VectorXd initQ(16);
    //initQ << 0,
          //0,
          //1.335,
          //0,
          //0,
          //0,
          //0,
          //0,
          //-0.392699,
          //0.785398,
          //1.1781,
          //0,
          //0,
          //-0.392699,
          //0.785398,
          //1.1781;
    //mParam->initQ = initQ;

    //std::unique_ptr<InvKinPlanner> mPlanner =
        //std::make_unique<InvKinPlanner>();

    //mPlanner->updatePlanningParameter(mParam);

    //double time(1.0);
    //Eigen::VectorXd pos, vel, trq;
    //mPlanner->getPlan(time, pos, vel, trq);

    //std::cout << "Done" << std::endl;
    //return 0;
/*}*/

int main(int argc, char *argv[])
{
    std::shared_ptr<InvKinPlannerParameter> mParam =
        std::make_shared<InvKinPlannerParameter>();
    mParam->paramSetFromYaml(THIS_COM"Config/Draco/PLANNER/ONE_STEP_PLANNING/CENTROID_PLANNER_RESULT.yaml");
    Eigen::VectorXd initQ(16);
    initQ << 0,
          0,
          1.335,
          0,
          0,
          0,
          0,
          0,
          -0.392699,
          0.785398,
          1.1781,
          0,
          0,
          -0.392699,
          0.785398,
          1.1781;
    mParam->initQ = initQ;

    std::unique_ptr<InvKinPlanner> mPlanner =
        std::make_unique<InvKinPlanner>();

    mPlanner->updatePlanningParameter(mParam);

    double time(1.0);
    Eigen::VectorXd pos, vel, trq;
    mPlanner->getPlan(time, pos, vel, trq);

    std::cout << "Done" << std::endl;
    return 0;
}
