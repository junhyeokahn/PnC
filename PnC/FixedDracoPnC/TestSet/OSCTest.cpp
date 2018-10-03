#include "PnC/FixedDracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/pseudo_inverse.hpp"

OSCTest::OSCTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller

    rf_pos_des_debug = Eigen::VectorXd::Zero(3);
    rf_vel_des_debug = Eigen::VectorXd::Zero(3);
    rf_acc_des_debug = Eigen::VectorXd::Zero(3);
    rf_pos_act_debug = Eigen::VectorXd::Zero(3);
    rf_vel_act_debug = Eigen::VectorXd::Zero(3);
    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&rf_pos_des_debug, VECT, "rf_pos_des_debug", 3);
    dataManager->RegisterData(&rf_vel_des_debug, VECT, "rf_vel_des_debug", 3);
    dataManager->RegisterData(&rf_acc_des_debug, VECT, "rf_acc_des_debug", 3);
    dataManager->RegisterData(&rf_pos_act_debug, VECT, "rf_pos_act_debug", 3);
    dataManager->RegisterData(&rf_vel_act_debug, VECT, "rf_vel_act_debug", 3);

    printf("[OSC Test] Constructed\n");
}

OSCTest::~OSCTest() {
}

void OSCTest::getTorqueInput(void * commandData_) {
    ///////////
    // Planning
    ///////////
    FixedDracoCommand* cmd = (FixedDracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    double t(mRobot->getTime());
    Eigen::VectorXd rf_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rf_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rf_vel_task = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rf_acc_des = Eigen::VectorXd::Zero(3);
    if (t < mTestInitTime + mInterpolationDuration) {
        static double d_ary[3];
        mSpline.getCurvePoint(t - mTestInitTime, d_ary);
        for (int i = 0; i < 3; ++i) rf_pos_des[i] = d_ary[i];
        mSpline.getCurveDerPoint(t - mTestInitTime, 1, d_ary);
        for (int i = 0; i < 3; ++i) rf_vel_des[i] = d_ary[i];
        mSpline.getCurveDerPoint(t - mTestInitTime, 2, d_ary);
        for (int i = 0; i < 3; ++i) rf_acc_des[i] = d_ary[i];
    } else {
        myUtils::getSinusoidTrajectory(mTestInitTime + mInterpolationDuration,
                mMid, mAmp, mFreq, t, rf_pos_des, rf_vel_des, rf_acc_des);
    }

    cmd->q.setZero();
    cmd->qdot.setZero();
    Eigen::VectorXd qddot_des(mRobot->getNumDofs());
    Eigen::VectorXd qdot_des(mRobot->getNumDofs());
    Eigen::VectorXd qddot_des_rf(mRobot->getNumDofs());
    Eigen::VectorXd qdot_des_rf(mRobot->getNumDofs());
    Eigen::VectorXd qddot_des_q(mRobot->getNumDofs());
    Eigen::VectorXd qdot_des_q(mRobot->getNumDofs());
    Eigen::MatrixXd J_rf = mRobot->getBodyNodeJacobian("rAnkle").block(3, 0, 3, mRobot->getNumDofs());
    Eigen::MatrixXd J_rf_bar(mRobot->getNumDofs(), 3);
    myUtils::weightedInverse(J_rf, mRobot->getInvMassMatrix(), J_rf_bar);
    Eigen::MatrixXd Jdot_rf = mRobot->getBodyNodeJacobianDot("rAnkle").block(3, 0, 3, mRobot->getNumDofs());
    Eigen::VectorXd rf_pos_act = mRobot->getBodyNodeIsometry("rAnkle").translation();
    Eigen::VectorXd rf_vel_act = mRobot->getBodyNodeSpatialVelocity("rAnkle").tail(3);
    for (int i = 0; i < 3; ++i) {
        rf_acc_des[i] += mKpRf[i] * (rf_pos_des[i] - rf_pos_act[i]) +
            mKdRf[i] * (rf_vel_des[i] - rf_vel_act[i]);
        rf_vel_task[i] = rf_vel_des[i] +
            mKpRf[i] * (rf_pos_des[i] - rf_pos_act[i]);
    }

    //////////
    // Control
    //////////

    // First Task
    qddot_des_rf = J_rf_bar * (rf_acc_des - Jdot_rf * mRobot->getQdot());
    //qddot_des_rf = J_rf_bar * (rf_acc_des);
    qdot_des_rf = J_rf_bar * (rf_vel_task);

    // Second Task
    Eigen::MatrixXd J_q = Eigen::MatrixXd::Identity(mRobot->getNumDofs(),
                                                    mRobot->getNumDofs());
    Eigen::MatrixXd N_rf =
        Eigen::MatrixXd::Identity(mRobot->getNumDofs(), mRobot->getNumDofs())
        - J_rf_bar * J_rf;
    Eigen::MatrixXd J_q_N_rf = J_q * N_rf;
    Eigen::MatrixXd J_q_N_rf_bar;
    myUtils::weightedInverse(J_q_N_rf, mRobot->getInvMassMatrix(), J_q_N_rf_bar);
    for (int i = 0; i < mRobot->getNumDofs(); ++i) {
        qddot_des_q[i] = mKpQ[i] * (mTestInitQ[i] - mRobot->getQ()[i]) +
            mKdQ[i] * (-mRobot->getQdot()[i]);
        qdot_des_q[i] = mKpQ[i] * (mTestInitQ[i] - mRobot->getQ()[i]);
    }
    qddot_des_q = J_q_N_rf * ( qddot_des_q );
    qdot_des_q = J_q_N_rf * ( qdot_des_q );

    // Compute Torque
    qddot_des = qddot_des_rf + qddot_des_q;
    qdot_des = qdot_des_rf + qdot_des_q;
    cmd->jtrq = mRobot->getMassMatrix() * qddot_des + mRobot->getGravity();

    // Admittance Ctrl
    // Option 1 : integrate, integrate
    //cmd->qdot = myUtils::eulerIntegration( mRobot->getQdot(), qddot_des, SERVO_RATE );
    //cmd->q = myUtils::eulerIntegration( mRobot->getQ(), cmd->qdot, SERVO_RATE );
    // TODO
    cmd->q = mTestInitQ;
    cmd->qdot.setZero();
    //cmd->jtrq = mRobot->getMassMatrix() * qddot_des + mRobot->getGravity();
    cmd->jtrq = mRobot->getMassMatrix() * qddot_des + mRobot->getGravity();
    // Option 2 : integrate
    //cmd->qdot = qdot_des;
    //cmd->q = myUtils::doubleIntegration(mRobot->getQ(), cmd->qdot, qddot_des, SERVO_RATE);

    rf_pos_des_debug = rf_pos_des;
    rf_vel_des_debug = rf_vel_des;
    rf_acc_des_debug = rf_acc_des;
    rf_pos_act_debug = rf_pos_act;
    rf_vel_act_debug = rf_vel_act;
}

void OSCTest::initialize() {
    mTestInitQ = mRobot->getQ();
    mTestInitRFPos = mRobot->getBodyNodeIsometry("rAnkle").translation();
    mTestInitTime = mRobot->getTime();

    //Planner Initialize
    try {
        YAML::Node test_cfg = YAML::LoadFile(THIS_COM"Config/FixedDraco/TEST/OSC_TEST.yaml");
        YAML::Node sinusoidal_cfg = test_cfg["sinusoidal_configuration"];
        myUtils::readParameter(sinusoidal_cfg, "transition_time", mInterpolationDuration);
        myUtils::readParameter(sinusoidal_cfg, "mid", mMid);
        mMid = mTestInitRFPos;
        mMid[0] += 0.02; //TODO
        mMid[2] += 0.05;
        myUtils::readParameter(sinusoidal_cfg, "amp", mAmp);
        myUtils::readParameter(sinusoidal_cfg, "freq", mFreq);
        YAML::Node control_cfg= test_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp_rf", mKpRf);
        myUtils::readParameter(control_cfg, "kd_rf", mKdRf);
        myUtils::readParameter(control_cfg, "kp_q", mKpQ);
        myUtils::readParameter(control_cfg, "kd_q", mKdQ);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
    double ini[9]; double fin[9]; double **middle_pt;
    for (int i = 0; i < 9; ++i) {
        ini[i] = 0.0;
        fin[i] = 0.0;
    }
    for (int i = 0; i < 3; ++i) {
        ini[i] = mTestInitRFPos[i];
        fin[i] = mMid[i];
    }
    for (int i = 3; i < 6; ++i) {
        fin[i] = mAmp[i-3]*2*M_PI*mFreq[i-3];
    }
    mSpline.SetParam(ini, fin, middle_pt, mInterpolationDuration);

    //Controller Initialize

    isInitialized = true;
}
