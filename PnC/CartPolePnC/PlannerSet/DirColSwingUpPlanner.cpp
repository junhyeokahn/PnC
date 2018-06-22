#include "CartPolePnC/PlannerSet/DirColSwingUpPlanner.hpp"
#include "Configuration.h"
#include "Utilities.hpp"

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

void DirColSwingUpPlanner::_evalTrajecotry( double time,
                                            Eigen::VectorXd & pos,
                                            Eigen::VectorXd & vel,
                                            Eigen::VectorXd acc,
                                            Eigen::VectorXd &eff ) {
  const drake::trajectories::PiecewisePolynomial<double> utraj =
      mDirCol->ReconstructInputTrajectory();
  const drake::trajectories::PiecewisePolynomial<double> xtraj =
      mDirCol->ReconstructStateTrajectory();
  if (xtraj.end_time() > time) {
      //pos = xtraj.value(time).block(0, 0, 2, 1);
      //vel = xtraj.value(time).block(0, 2, 2, 1);
      pos[0] = (xtraj.value(time))(0, 0);
      pos[1] = (xtraj.value(time))(0, 1);
      vel[0] = (xtraj.value(time))(0, 2);
      vel[1] = (xtraj.value(time))(0, 3);
      eff = utraj.value(time);
  } else {
      pos.setZero();
      vel.setZero();
      eff.setZero();
      exit(0);
  }

  // Print Solution Trajectories
  _saveTrajectory();
}

void DirColSwingUpPlanner::_saveTrajectory() {
    const drake::trajectories::PiecewisePolynomial<double> utraj =
        mDirCol->ReconstructInputTrajectory();
    const drake::trajectories::PiecewisePolynomial<double> xtraj =
        mDirCol->ReconstructStateTrajectory();
    double startTime = xtraj.start_time();
    double endTime = xtraj.end_time();
    int numEval = std::floor((endTime - startTime) / SERVO_RATE);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd evalTime = Eigen::VectorXd::Zero(1);
    evalTime[0] = startTime;
    for (int i = 0; i < numEval; ++i) {
        x = xtraj.value(evalTime[0]);
        u = utraj.value(evalTime[0]);
        myUtils::saveVector(x, "planned_state");
        myUtils::saveVector(u, "planned_input");
        myUtils::saveVector(evalTime, "time");
        evalTime[0] += SERVO_RATE;
    }

    std::cout << "Trajectory Saved" << std::endl;
    exit(0);
}
