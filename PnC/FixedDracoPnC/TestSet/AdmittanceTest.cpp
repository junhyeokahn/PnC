#include "PnC/FixedDracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"
#include <random>

AdmittanceTest::AdmittanceTest(RobotSystem* robot_): Test(robot_),
                                                     mDoUpdate(true),
                                                     mTransTime(0.0),
                                                     mStayTime(0.0),
                                                     mIsSaved(false),
                                                     mTestInitTime(0.0),
                                                     mSplineInitTime(0.0) {
    // Choose Planner

    // Choose Controller

    aug_T_j_com_list.reserve(50000);
    aug_R_w_com_list.reserve(50000);

    printf("[Admittance Test] Constructed\n");
}

AdmittanceTest::~AdmittanceTest() {
}

void AdmittanceTest::getTorqueInput(void * commandData_) {
    FixedDracoCommand* cmd = (FixedDracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    if (mDoUpdate) { _updateSpline(); }

    Eigen::VectorXd q_des = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(10);

    double t(mRobot->getTime());
    if (t < mSplineInitTime + mTransTime) {
        static double d_ary[10];
        mSpline.getCurvePoint(t - mSplineInitTime, d_ary);
        for (int i = 0; i < 10; ++i) q_des[i] = d_ary[i];
        mSpline.getCurveDerPoint(t - mSplineInitTime, 1, d_ary);
        for (int i = 0; i < 10; ++i) qdot_des[i] = d_ary[i];
        mSpline.getCurveDerPoint(t - mSplineInitTime, 2, d_ary);
        for (int i = 0; i < 10; ++i) qddot_des[i] = d_ary[i];
    } else if (t < mSplineInitTime + mTransTime + mStayTime) {
        q_des = mStayPosition;
        qdot_des.setZero();
        qddot_des.setZero();
    } else {
        mDoUpdate = true;
    }

    cmd->q = q_des;
    cmd->qdot = qdot_des;
    for (int i = 0; i < 10; ++i) {
        qddot_des[i] += mKp[i] * (q_des[i] - mRobot->getQ()[i]) +
            mKd[i] * (qdot_des[i] - mRobot->getQdot()[i]);
    }
    cmd->jtrq = mRobot->getMassMatrix() * qddot_des + mRobot->getGravity();

    Eigen::VectorXd nearly_zero = Eigen::VectorXd::Constant(10, 0.0001);
    if (myUtils::isInBoundingBox(-nearly_zero, mRobot->getQdot(), nearly_zero)) {
        _collectData();
    }

    if (!mIsSaved && mRobot->getTime() > 10) {
        _dataSave();
        mIsSaved = true;
    }
}

void AdmittanceTest::initialize() {
    mTestInitQ = mRobot->getQ();
    mTestInitTime = mRobot->getTime();

    //Planner Initialize
    try {
        YAML::Node test_cfg = YAML::LoadFile(THIS_COM"Config/FixedDraco/TEST/ADMITTANCE_TEST.yaml");
        YAML::Node planning_cfg = test_cfg["planning_configuration"];
        myUtils::readParameter(planning_cfg, "transition_time", mTransTime);
        myUtils::readParameter(planning_cfg, "stay_time", mStayTime);
        myUtils::readParameter(planning_cfg, "joint_lb", mLb);
        myUtils::readParameter(planning_cfg, "joint_ub", mUb);
        YAML::Node control_cfg= test_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }

    //Controller Initialize

    isInitialized = true;
}

void AdmittanceTest::_updateSpline() {

    Eigen::VectorXd current_point = mRobot->getQ();
    mStayPosition = Eigen::VectorXd::Zero(mLb.size());

    for (int i = 0; i < mLb.size(); ++i) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(mLb[i], mUb[i]);
        mStayPosition[i] = dis(gen);
    }

    double ini[30];
    double fin[30];
    double **middle_pt;
    for (int i = 0; i < 30; ++i) {
        ini[i] = 0.0;
        fin[i] = 0.0;
    }
    for (int i = 0; i < 10; ++i) {
        ini[i] = current_point[i];
        fin[i] = mStayPosition[i];
    }
    mSpline.SetParam(ini, fin, middle_pt, mTransTime);

    mSplineInitTime = mRobot->getTime();
    mDoUpdate = false;
}

void AdmittanceTest::_collectData() {
    int num_joint(5);
    int start_idx(6);
    std::vector<std::string> body_node_name = {"rHipYaw", "rHipRoll", "rHipPitch", "rKnee", "rAnkle"};
    Eigen::MatrixXd aug_T_j_com = Eigen::MatrixXd::Zero(4*num_joint, 4*num_joint);
    Eigen::MatrixXd aug_R_w_com = Eigen::MatrixXd::Zero(3*num_joint, 3);
    for (int i = 0; i < num_joint; ++i) {
        for (int j = i; j < num_joint; ++j) {
            aug_T_j_com.block(4*i, 4*j, 4, 4) =
                (mRobot->getBodyNodeIsometry(body_node_name[i]).inverse() *
                mRobot->getBodyNodeCoMIsometry(body_node_name[j]) ).matrix();
        }
        aug_R_w_com.block(3*i, 0, 3, 3) =
            mRobot->getBodyNodeCoMIsometry(body_node_name[i]).linear();
    }
    aug_T_j_com_list.push_back(aug_T_j_com);
    aug_R_w_com_list.push_back(aug_R_w_com);
}

void AdmittanceTest::_dataSave() {
    std::cout << "start saving data" << std::endl;
    int num_data(aug_T_j_com_list.size());
    try {

        YAML::Node sys_id_cfg;
        for (int i = 0; i < num_data; ++i) {
            sys_id_cfg["data"+std::to_string(i)]["aug_T_j_com"] = aug_T_j_com_list[i];
            sys_id_cfg["data"+std::to_string(i)]["aug_R_w_com"] = aug_R_w_com_list[i];
        }

        std::ofstream file_out(THIS_COM"Config/FixedDraco/SysID/MASS.yaml");
        file_out << sys_id_cfg;
    } catch (YAML::ParserException &e) { std::cout << e.what() << "\n"; }

    exit(0);
}
