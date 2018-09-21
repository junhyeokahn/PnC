#include "PnC/FixedDracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"

JointTest::JointTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller

    printf("[Joint Test] Constructed\n");
}

JointTest::~JointTest() {
}

void JointTest::getTorqueInput(void * commandData_) {
    FixedDracoCommand* cmd = (FixedDracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    double t(mRobot->getTime());
    Eigen::VectorXd q_des = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(10);
    if (t < mTestInitTime + mInterpolationDuration) {
        static double d_ary[10];
        mSpline.getCurvePoint(t - mTestInitTime, d_ary);
        for (int i = 0; i < 10; ++i) q_des[i] = d_ary[i];
        mSpline.getCurveDerPoint(t - mTestInitTime, 1, d_ary);
        for (int i = 0; i < 10; ++i) qdot_des[i] = d_ary[i];
        mSpline.getCurveDerPoint(t - mTestInitTime, 2, d_ary);
        for (int i = 0; i < 10; ++i) qddot_des[i] = d_ary[i];
    } else {
        myUtils::getSinusoidTrajectory(mTestInitTime + mInterpolationDuration,
                mMid, mAmp, mFreq, t, q_des, qdot_des, qddot_des);
    }

    cmd->q = q_des;
    cmd->qdot = qdot_des;
    for (int i = 0; i < 10; ++i) {
        qddot_des[i] += mKp[i] * (q_des[i] - mRobot->getQ()[i]) +
            mKd[i] * (qdot_des[i] - mRobot->getQdot()[i]);
    }
    cmd->jtrq = mRobot->getMassMatrix() * qddot_des + mRobot->getGravity();

}

void JointTest::initialize() {
    mTestInitQ = mRobot->getQ();
    mTestInitTime = mRobot->getTime();

    //Planner Initialize
    try {
        YAML::Node test_cfg = YAML::LoadFile(THIS_COM"Config/FixedDraco/TEST/JOINT_TEST.yaml");
        YAML::Node sinusoidal_cfg = test_cfg["sinusoidal_configuration"];
        myUtils::readParameter(sinusoidal_cfg, "transition_time", mInterpolationDuration);
        myUtils::readParameter(sinusoidal_cfg, "mid", mMid);
        myUtils::readParameter(sinusoidal_cfg, "amp", mAmp);
        myUtils::readParameter(sinusoidal_cfg, "freq", mFreq);
        YAML::Node control_cfg= test_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
    double ini[30];
    double fin[30];
    double **middle_pt;
    for (int i = 0; i < 30; ++i) {
        ini[i] = 0.0;
        fin[i] = 0.0;
    }
    for (int i = 0; i < 10; ++i) {
        ini[i] = mTestInitQ[i];
        fin[i] = mMid[i];
    }
    for (int i = 10; i < 20; ++i) {
        fin[i] = mAmp[i-10]*2*M_PI*mFreq[i-10];
    }
    mSpline.SetParam(ini, fin, middle_pt, mInterpolationDuration);

    //Controller Initialize

    isInitialized = true;
}
