#include "CartPolePnC/TestSet/TestSet.hpp"
#include "CartPolePnC/PlannerSet/PlannerSet.hpp"
#include "RobotSystem.hpp"

DirColSwingUpTest::DirColSwingUpTest(RobotSystem* robot_): Test(robot_) {
    mPlanner = new DirColSwingUpPlanner();
    printf("[DirCol Swing Up Test] Constructed\n");
}

DirColSwingUpTest::~DirColSwingUpTest() {
    delete mPlanner;
}

Eigen::VectorXd DirColSwingUpTest::getTorqueInput() {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd pos = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd eff = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    mPlanner->getPlan(mRobot->getTime(), pos, vel, acc, eff);

    ret = eff;

    return ret;
}

void DirColSwingUpTest::initialize() {
    // Update Planning Parameter
    std::shared_ptr<DirColSwingUpPlanningParameter> param =
        std::make_shared<DirColSwingUpPlanningParameter>();
    Eigen::VectorXd state(2*mRobot->getNumDofs());
    state << mRobot->getQ(), mRobot->getQdot();
    // TODO: use a yaml
    param->initialState = Eigen::VectorXd::Zero(4);
    param->finalState = Eigen::Vector4d(0., M_PI, 0.,  0.);
    param->numTimeSample = 21;
    param->minimumTimeStep = 0.1;
    param->maximumTimeStep = 0.4;
    param->startTime = 0.0;
    param->endTime = 100.0;
    param->torqueLimit = 100.0;
    param->R = 10.;
    param->timeSpanInit = 4.0;
    mPlanner->updatePlanningParameter(param);

    isInitialized = true;
}
