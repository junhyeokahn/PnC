#include "PnC/CartPolePnC/PlannerSet/DirColSwingUpPlanner.hpp"
#include "Configuration.h"
#include "Utils/Utilities.hpp"

DirColSwingUpPlanner::DirColSwingUpPlanner() : Planner() {
    mTree = std::make_unique< RigidBodyTree<double> >();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            THIS_COM"RobotSystem/RobotModel/Robot/CartPole/CartPole.urdf",
            drake::multibody::joints::kFixed, mTree.get());
    mPlant = std::make_unique< drake::systems::RigidBodyPlant<double> >(
            std::move(mTree), 0.0);

    printf("[DirCol Swing Up Planner] Constructed\n");
}

DirColSwingUpPlanner::~DirColSwingUpPlanner() {
}

void DirColSwingUpPlanner::_doPlan() {
    // =========================================
    // Construct Direct collocation Optimization
    // =========================================
    std::unique_ptr<drake::systems::Context<double>> context =
        mPlant->CreateDefaultContext();
    std::shared_ptr<DirColSwingUpPlanningParameter> param =
        std::dynamic_pointer_cast<DirColSwingUpPlanningParameter>(mParam);
    mDirCol = std::make_unique<drake::systems::trajectory_optimization::DirectCollocation>(
                mPlant.get(), *context, param->numTimeSample,
                param->minimumTimeStep, param->maximumTimeStep);
    // ==============
    // Add Constraint
    // ==============
    mDirCol->AddEqualTimeIntervalsConstraints();
    // Input Constraint
    const drake::solvers::VectorXDecisionVariable& u = mDirCol->input();
    mDirCol->AddConstraintToAllKnotPoints(-(param->torqueLimit) <= u(0));
    mDirCol->AddConstraintToAllKnotPoints(u(0) <= param->torqueLimit);
    // Initial and Final State Constraint
    mDirCol->AddLinearConstraint(mDirCol->initial_state() ==
            param->initialState);
    mDirCol->AddLinearConstraint(mDirCol->final_state() ==
            param->finalState);
    // ========
    // Add Cost
    // ========
    // Add Running Cost
    mDirCol->AddRunningCost((param->R * u(0)) * u(0));
    // Add Final Cost
    mDirCol->AddFinalCost(mDirCol->time().cast<drake::symbolic::Expression>());

    // ========================
    // Throw Initial Trajectory
    // ========================
    auto traj_init_x = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            {0, param->timeSpanInit}, {param->initialState, param->finalState});
    mDirCol->SetInitialTrajectory(
            drake::trajectories::PiecewisePolynomial<double>(), traj_init_x);
    drake::solvers::SolutionResult result = mDirCol->Solve();
    if (result != drake::solvers::SolutionResult::kSolutionFound) {
        std::cerr << "Failed to solve optimization for the swing-up trajectory"
            << std::endl;
        exit(0);
    }
}

void DirColSwingUpPlanner::_evalTrajectory( double time,
        Eigen::VectorXd & pos,
        Eigen::VectorXd & vel,
        Eigen::VectorXd & trq ) {
    const drake::trajectories::PiecewisePolynomial<double> utraj =
        mDirCol->ReconstructInputTrajectory();
    const drake::trajectories::PiecewisePolynomial<double> xtraj =
        mDirCol->ReconstructStateTrajectory();

    mParam->startTime = xtraj.start_time();
    mParam->endTime = xtraj.end_time();

    if (xtraj.end_time() > time) {
        pos[0] = (xtraj.value(time))(0, 0);
        pos[1] = (xtraj.value(time))(1, 0);
        vel[0] = (xtraj.value(time))(2, 0);
        vel[1] = (xtraj.value(time))(3, 0);
        trq = utraj.value(time);
    } else {
        pos.setZero();
        vel.setZero();
        trq.setZero();
        std::cout << "Try to evaluate trajectory at the wrong time" << std::endl;
        exit(0);
    }
}
