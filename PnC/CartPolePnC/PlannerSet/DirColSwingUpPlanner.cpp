#include "CartPolePnC/PlannerSet/DirColSwingUpPlanner.hpp"
#include "Configuration.h"

DirColSwingUpPlanner::DirColSwingUpPlanner() : Planner() {
    mTree = std::make_unique< RigidBodyTree<double> >();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            THIS_COM"Simulator/SimulationModel/RobotModel/CartPole/CartPole.urdf",
            drake::multibody::joints::kFixed, mTree.get());
    mPlant = std::make_unique< drake::systems::RigidBodyPlant<double> >(
            std::move(mTree), 0.0);

    printf("[DirCol Swing Up Test] Constructed\n");
}

DirColSwingUpPlanner::~DirColSwingUpPlanner() {
}

void DirColSwingUpPlanner::_doPlan() {
    std::unique_ptr<drake::systems::Context<double>> context =
        mPlant->CreateDefaultContext();
    std::shared_ptr<DirColSwingUpPlanningParameter> param =
        std::dynamic_pointer_cast<DirColSwingUpPlanningParameter>(mParam);
    mDirCol = std::make_unique<drake::systems::trajectory_optimization::DirectCollocation>(
                mPlant.get(), *context, param->numTimeSample,
                param->minimumTimeStep, param->maximumTimeStep);
    mDirCol->AddEqualTimeIntervalsConstraints();
    const drake::solvers::VectorXDecisionVariable& u = mDirCol->input();
    // Input Constraint
    mDirCol->AddConstraintToAllKnotPoints(-(param->torqueLimit) <= u(0));
    mDirCol->AddConstraintToAllKnotPoints(u(0) <= param->torqueLimit);
    // Initial Final Constraint
    mDirCol->AddLinearConstraint(mDirCol->initial_state() ==
            param->initialState);
    mDirCol->AddLinearConstraint(mDirCol->final_state() == 
            param->finalState);
    // Add Running Cost
    mDirCol->AddRunningCost((param->R * u(0)) * u(0));
    // Add Final Cost
    mDirCol->AddFinalCost(mDirCol->time().cast<drake::symbolic::Expression>());

    auto traj_init_x = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            {0, param->timeSpanInit}, {param->initialState, param->finalState});
    mDirCol->SetInitialTrajectory(
            drake::trajectories::PiecewisePolynomial<double>(), traj_init_x);
    drake::solvers::SolutionResult result = mDirCol->Solve();
    if (result != drake::solvers::SolutionResult::kSolutionFound) {
        std::cerr << "Failed to solve optimization for the swing-up trajectory"
            << std::endl;
    }
}

void DirColSwingUpPlanner::_evalTrajecotry( double time,
                                            Eigen::VectorXd & pos,
                                            Eigen::VectorXd & vel,
                                            Eigen::VectorXd acc,
                                            Eigen::VectorXd &eff ) {
  const drake::trajectories::PiecewisePolynomial<double> utraj =
      mDirCol->ReconstructInputTrajectory();
  const drake::trajectories::PiecewisePolynomial<double> xtraj =
      mDirCol->ReconstructStateTrajectory();
}
