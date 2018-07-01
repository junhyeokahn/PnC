#include "CartPolePnC/TestSet/TestSet.hpp"
#include "CartPolePnC/PlannerSet/PlannerSet.hpp"
#include "RobotSystem.hpp"
#include "DataManager.hpp"
#include "ParamHandler.hpp"

DirColSwingUpTest::DirColSwingUpTest(RobotSystem* robot_): Test(robot_) {
    mPlanner = new DirColSwingUpPlanner();

    printf("[DirCol Swing Up Test] Constructed\n");
}

DirColSwingUpTest::~DirColSwingUpTest() {
    delete mPlanner;
}

void DirColSwingUpTest::getTorqueInput(void* commandData_) {

    CartPoleCommand* cmd = (CartPoleCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    mPlanner->getPlan(mRobot->getTime(), cmd->q,
                                         cmd->qdot,
                                         cmd->jtrq);
}

void DirColSwingUpTest::initialize() {
    // Update Planning Parameter
    std::shared_ptr<DirColSwingUpPlanningParameter> param =
        std::make_shared<DirColSwingUpPlanningParameter>();

    param->numState = 4;
    param->numInput = 1;
    ParamHandler handler(THIS_COM"Config/CartPole/DIRCOLSWINGUP.yaml");
    double tmp_double;
    std::vector<double> tmp_vector;
    tmp_vector.resize(param->numState);
    handler.getVector("InitialState", tmp_vector);
    param->initialState = Eigen::Vector4d(tmp_vector[0], tmp_vector[1],
                                          tmp_vector[2], tmp_vector[3]);
    handler.getVector("FinalState", tmp_vector);
    param->finalState= Eigen::Vector4d(tmp_vector[0], tmp_vector[1],
                                          tmp_vector[2], tmp_vector[3]);
    handler.getValue("NumTimeSample", tmp_double);
    param->numTimeSample = tmp_double;
    handler.getValue("MinimumTimeStep", tmp_double);
    param->minimumTimeStep = tmp_double;
    handler.getValue("MaximumTimeStep", tmp_double);
    param->maximumTimeStep = tmp_double;
    handler.getValue("StartTime", tmp_double);
    param->startTime = tmp_double;
    handler.getValue("EndTime", tmp_double);
    param->endTime = tmp_double;
    handler.getValue("TorqueLimit", tmp_double);
    param->torqueLimit = tmp_double;
    handler.getValue("InputCost", tmp_double);
    param->R = tmp_double;
    handler.getValue("TimeSpanInit", tmp_double);
    param->timeSpanInit = tmp_double;
    mPlanner->updatePlanningParameter(param);

    isInitialized = true;
}
