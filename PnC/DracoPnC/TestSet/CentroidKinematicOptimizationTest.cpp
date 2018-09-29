#include "PnC/DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"
#include "PnC/WBC/WBLC/WBLC.hpp"
#include "PnC/WBC/WBLC/WBLCContact.hpp"

CentroidKinematicOptimizationTest::CentroidKinematicOptimizationTest(RobotSystem* robot_): Test(robot_) {
    try {
        YAML::Node test_cfg =
            YAML::LoadFile(THIS_COM"Config/Draco/TEST/CENTROID_KINEMATIC_TEST.yaml");
        YAML::Node control_cfg = test_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "centroid_task_kp", mCentroidTaskKp);
        myUtils::readParameter(control_cfg, "centroid_task_kd", mCentroidTaskKd);
        myUtils::readParameter(control_cfg, "joint_task_kp", mJointTaskKp);
        myUtils::readParameter(control_cfg, "joint_task_kd", mJointTaskKd);
        YAML::Node planner_cfg = test_cfg["planner_configuration"];
        myUtils::readParameter(planner_cfg, "transition_time", mInterpolationDuration);
        myUtils::readParameter(planner_cfg, "nominal_centroid_state", mNominalCentroidState);
        myUtils::readParameter(planner_cfg, "nominal_joint_state", mNominalJointState);
        myUtils::readParameter(planner_cfg, "pre_planned_file", mPrePlannedFile);
        myUtils::readParameter(planner_cfg, "foot_swing_height", mFootSwingHeight);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }

    // Choose Planner
    //mPlanner = new PrePlannedCentroidPlanner();
    mPlanner = std::make_unique<PrePlannedCentroidPlanner>();
    mPlanningParam = std::make_shared<PrePlannedCentroidPlannerParameter>();
    mPlanningParam->trajectoryFile = THIS_COM+mPrePlannedFile;
    mPlanningParam->swingHeight = mFootSwingHeight;
    mPlanner->updatePlanningParameter(mPlanningParam);
    Eigen::VectorXd p, v, a;
    mPlanner->getPlan(0.1, p, v, a);

    // Choose Controller
    std::vector<bool> act_list;
    act_list.resize(mRobot->getNumDofs(), true);
    for(int i(0); i < mRobot->getNumVirtualDofs(); ++i)
        act_list[i] = false;
    mWBLC = new WBLC(act_list);
    mWBLCExtraData= new WBLC_ExtraData();

    mRfContact = new WBLCContact(mRobot, "rAnkle", 0.7);
    mLfContact = new WBLCContact(mRobot, "lAnkle", 0.7);
    mContactList.clear();
    mContactList.push_back(mRfContact);
    mContactList.push_back(mLfContact);
    mCentroidTask = new Task(mRobot, TaskType::CENTROID);
    mCentroidTask->setGain(mCentroidTaskKp, mCentroidTaskKd);
    mJointTask = new Task(mRobot, TaskType::JOINT);
    mJointTask->setGain(mJointTaskKp, mJointTaskKd);
    mTaskList.clear();
    mTaskList.push_back(mCentroidTask);
    mTaskList.push_back(mJointTask);
    mCentroidPosDes = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    mCentroidVelDes = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    mCentroidAccDes = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    mJointPosDes = Eigen::VectorXd::Zero(mTaskList[1]->getDims());
    mJointVelDes = Eigen::VectorXd::Zero(mTaskList[1]->getDims());
    mJointAccDes = Eigen::VectorXd::Zero(mTaskList[1]->getDims());

    printf("[Centroid Kinematic Optimization Test] Constructed\n");
}

CentroidKinematicOptimizationTest::~CentroidKinematicOptimizationTest() {
    delete mWBLC;
    delete mWBLCExtraData;
    delete mJointTask;
    delete mRfContact;
    delete mLfContact;
    //delete mPlanner;
}

void CentroidKinematicOptimizationTest::getTorqueInput(void * commandData_) {

    DracoCommand* cmd = (DracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    ///////////////////////////////////////////////////////////////////////////
    // Planner
    ///////////////////////////////////////////////////////////////////////////

    double t(mRobot->getTime());
    // Centroid Task
    if (t < mTestInitTime + mInterpolationDuration) {
        for (int i = 0; i < 3; ++i) {
            mCentroidPosDes[i+3] = myUtils::smooth_changing(mTestInitCoMPos[i],
                    mNominalCentroidState[i+3], mInterpolationDuration, t);
            mCentroidVelDes[i+3] = mRobot->getRobotMass() * myUtils::smooth_changing_vel(mTestInitCoMPos[i],
                    mNominalCentroidState[i+3], mInterpolationDuration, t);
            mCentroidAccDes[i+3] = mRobot->getRobotMass() * myUtils::smooth_changing_acc(mTestInitCoMPos[i],
                    mNominalCentroidState[i+3], mInterpolationDuration, t);
        }
    } else {
        Eigen::VectorXd p_ang_lin_rf_lf, v_ang_lin_rf_lf, a_ang_lin_rf_lf;
        mPlanner->getPlan( t - mTestInitTime - mInterpolationDuration,
                p_ang_lin_rf_lf, v_ang_lin_rf_lf, a_ang_lin_rf_lf);
        mCentroidPosDes = p_ang_lin_rf_lf.segment(0, 6);
        mCentroidVelDes = v_ang_lin_rf_lf.segment(0, 6);
        mCentroidAccDes = a_ang_lin_rf_lf.segment(0, 6);
        // foot task
        //mCentroidPosDes = mNominalCentroidState;
        //mCentroidVelDes.setZero();
        //mCentroidAccDes.setZero();
    }

    // 2. Joint Task
    if(t < mTestInitTime + mInterpolationDuration) {
        for (int i = 0; i < mTaskList[1]->getDims(); ++i) {
            mJointPosDes[i] = myUtils::smooth_changing(mTestInitQ[i],
                    mNominalJointState[i], mInterpolationDuration, t);
            mJointVelDes[i] = myUtils::smooth_changing_vel(mTestInitQ[i],
                    mNominalJointState[i], mInterpolationDuration, t);
            mJointAccDes[i] = myUtils::smooth_changing_acc(mTestInitQ[i],
                    mNominalJointState[i], mInterpolationDuration, t);
        }
    } else {
        mJointPosDes = mNominalJointState;
        mJointVelDes.setZero();
        mJointAccDes.setZero();
    }

    ///////////////////////////////////////////////////////////////////////////
    // Controller
    ///////////////////////////////////////////////////////////////////////////

    // Whole Body Admitance Control
    _updateContact();
    _updateTask();
    _WBLCpreProcess();
    mWBLC->MakeTorque(mTaskList, mContactList, cmd->jtrq, mWBLCExtraData);
    _WBLCpostProcess();

    // Admittance Option 1
    //Eigen::VectorXd qddot_des = mWBLC->getQddot();
    //Eigen::VectorXd qdot_des = myUtils::eulerIntegration(mRobot->getQdot(), qddot_des, SERVO_RATE);
    //Eigen::VectorXd q_des = myUtils::eulerIntegration(mRobot->getQ(), qdot_des, SERVO_RATE);
    //cmd->q = q_des;
    //cmd->qdot = qdot_des;
    // Admittance Option 2 : seems different from option 1 due to delta
    cmd->qdot = mWBLC->getQdot();
    cmd->q = myUtils::doubleIntegration(mRobot->getQ(), mWBLC->getQdot(), mWBLC->getQddot(), SERVO_RATE);
}

void CentroidKinematicOptimizationTest::initialize() {
    //Planner Initialize
    mTestInitQ = mRobot->getQ();
    mTestInitCoMPos = mRobot->getCoMPosition();
    mTestInitTime = mRobot->getTime();

    //Controller Initialize

    isInitialized = true;
}

void CentroidKinematicOptimizationTest::_updateTask() {
    mTaskList[0]->updateTaskSpec(mCentroidPosDes, mCentroidVelDes, mCentroidAccDes);
    mTaskList[1]->updateTaskSpec(mJointPosDes, mJointVelDes, mJointAccDes); // Any way no left dimension
}


void CentroidKinematicOptimizationTest::_updateContact() {
    for (int i = 0; i < mContactList.size(); ++i)
        mContactList[i]->updateWBLCContactSpec();
}

void CentroidKinematicOptimizationTest::_WBLCpostProcess() {
    for(int i(0); i<mTaskList.size(); ++i)
        mTaskList[i]->unsetTaskSpec();
    for(int i(0); i<mContactList.size(); ++i)
        mContactList[i]->unsetWBLCContactSpec();
}

void CentroidKinematicOptimizationTest::_WBLCpreProcess() {
    // dynamic property setting
    mWBLC->UpdateSetting(mRobot->getMassMatrix(),
                         mRobot->getInvMassMatrix(),
                         mRobot->getCoriolis(),
                         mRobot->getGravity());
    // cost setting
    int taskDim(0);
    int contactDim(0);
    taskDim += mTaskList[0]->getDims();
    for (int i = 0; i < mContactList.size(); ++i)
        contactDim += 6;
    mWBLCExtraData->cost_weight = Eigen::VectorXd::Zero(taskDim + contactDim);

    for(int i(0); i<taskDim; ++i) {
        mWBLCExtraData->cost_weight[i] = 1000000.0;
    }
    for(int i(0); i<contactDim; ++i){
        mWBLCExtraData->cost_weight[taskDim + i] = 1.0;
    }
    mWBLCExtraData->cost_weight[taskDim + 5] = 0.001; // vertical reaction
    mWBLCExtraData->cost_weight[taskDim + 11] = 0.001; // vertical reaction
}

