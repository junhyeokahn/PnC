#include <stdio.h>
#include <iostream>
#include <memory>
#include "CartPolePnC/PlannerSet/PlannerSet.hpp"
#include "ParamHandler.hpp"
#include "drake/solvers/mosek_solver.h"

void planDirColSwingUp() {
    // Construct Planner and Parameters
    auto Planner = std::make_unique<DirColSwingUpPlanner>();
    auto Param = std::make_shared<DirColSwingUpPlanningParameter>();

    // Set Planning Parameters
    Param->numState = 4;
    Param->numInput = 1;
    ParamHandler handler(THIS_COM"Config/CartPole/DIRCOLSWINGUP.yaml");
    double tmp_double;
    std::vector<double> tmp_vector;
    tmp_vector.resize(Param->numState);
    handler.getVector("InitialState", tmp_vector);
    Param->initialState = Eigen::Vector4d(tmp_vector[0], tmp_vector[1],
                                          tmp_vector[2], tmp_vector[3]);
    handler.getVector("FinalState", tmp_vector);
    Param->finalState= Eigen::Vector4d(tmp_vector[0], tmp_vector[1],
                                          tmp_vector[2], tmp_vector[3]);
    handler.getValue("NumTimeSample", tmp_double);
    Param->numTimeSample = tmp_double;
    handler.getValue("MinimumTimeStep", tmp_double);
    Param->minimumTimeStep = tmp_double;
    handler.getValue("MaximumTimeStep", tmp_double);
    Param->maximumTimeStep = tmp_double;
    handler.getValue("StartTime", tmp_double);
    Param->startTime = tmp_double;
    handler.getValue("EndTime", tmp_double);
    Param->endTime = tmp_double;
    handler.getValue("TorqueLimit", tmp_double);
    Param->torqueLimit = tmp_double;
    handler.getValue("InputCost", tmp_double);
    Param->R = tmp_double;
    handler.getValue("TimeSpanInit", tmp_double);
    Param->timeSpanInit = tmp_double;
    Planner->updatePlanningParameter(Param);

    // Save Trajectory
    Planner->saveTrajectory("PnC/CartPolePnC/OfflineTrajectoryGeneration/TrajectoriesBin/dircol_swing_up");
}

int main(int argc, char *argv[])
{
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  std::cout << mosek_license << std::endl;
  //exit(0);
    ParamHandler handler(THIS_COM"Config/CartPole/OFFLINETRAJECTORYGENERATION.yaml");
    std::string PlannerName;
    handler.getString("PlannerName", PlannerName);
    if (PlannerName == "DirColSwingUp") {
        planDirColSwingUp();
    } else {
        std::cout << "[No such Planner]" << std::endl;
    }

    return 0;
}
