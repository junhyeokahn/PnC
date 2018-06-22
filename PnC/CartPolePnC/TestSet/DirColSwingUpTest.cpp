#include "CartPolePnC/TestSet/TestSet.hpp"
#include "CartPolePnC/PlannerSet/PlannerSet.hpp"
#include "RobotSystem.hpp"
#include "DataManager.hpp"

DirColSwingUpTest::DirColSwingUpTest(RobotSystem* robot_): Test(robot_) {
    mPlanner = new DirColSwingUpPlanner();

    DataManager* dataManager = DataManager::GetDataManager();
    mPosDes = Eigen::VectorXd::Zero(2);
    mVelDes = Eigen::VectorXd::Zero(2);
    mEffDes = 0.0;
    mPosAct = Eigen::VectorXd::Zero(2);
    mVelAct = Eigen::VectorXd::Zero(2);
    dataManager->RegisterData(&mPosDes, VECT2, "JPosDes", 2);
    dataManager->RegisterData(&mVelDes, VECT2, "JVelDes", 2);
    dataManager->RegisterData(&mEffDes, DOUBLE, "JEffDes");
    dataManager->RegisterData(&mPosAct, VECT2, "JPosAct", 2);
    dataManager->RegisterData(&mVelAct, VECT2, "JVelAct", 2);

    printf("[DirCol Swing Up Test] Constructed\n");
}

DirColSwingUpTest::~DirColSwingUpTest() {
    delete mPlanner;
}

Eigen::VectorXd DirColSwingUpTest::getTorqueInput() {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd pos = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    Eigen::VectorXd eff = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    mPlanner->getPlan(mRobot->getTime(), pos, vel, acc, eff);

    ret = eff;

    //Register Data
    mPosDes = pos;
    mVelDes = vel;
    mEffDes = eff[0];
    mPosAct = mRobot->getQ();
    mVelAct = mRobot->getQdot();
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << mPosDes<< std::endl;
    std::cout << "--" << std::endl;
    std::cout << mVelDes<< std::endl;
    std::cout << "--" << std::endl;
    std::cout << mPosAct << std::endl;
    std::cout << "--" << std::endl;
    std::cout << mEffDes<< std::endl;
    std::cout << "--" << std::endl;

    return ret;
}

void DirColSwingUpTest::initialize() {
    // Update Planning Parameter
    std::shared_ptr<DirColSwingUpPlanningParameter> param =
        std::make_shared<DirColSwingUpPlanningParameter>();
    Eigen::VectorXd state(2*mRobot->getNumDofs());
    state << mRobot->getQ(), mRobot->getQdot();
    // TODO: use a yaml
    param->initialState = Eigen::Vector4d(0., M_PI, 0.,  0.);
    param->finalState = Eigen::VectorXd::Zero(4);
    param->numTimeSample = 21;
    param->minimumTimeStep = 0.1;
    param->maximumTimeStep = 0.4;
    param->startTime = mRobot->getTime();
    param->endTime = 100.0;
    param->torqueLimit = 40.0;
    param->R = 10.;
    param->timeSpanInit = 4.0;
    mPlanner->updatePlanningParameter(param);

    isInitialized = true;
}
