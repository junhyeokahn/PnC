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
                                                     mIsCollect(true),
                                                     mTestInitTime(0.0),
                                                     mSplineInitTime(0.0),
                                                     mIsVirtualUpdated(false) {

    mVirtualJPos = Eigen::VectorXd::Zero(10);
    mVirtualJVel = Eigen::VectorXd::Zero(10);
    // Choose Planner

    // Choose Controller
    body_node_name = {"rHipYaw", "rHipRoll", "rHipPitch", "rKnee", "rAnkle"};

    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mVirtualJPos, VECT, "VirtualJPos", 10);
    dataManager->RegisterData(&mVirtualJVel, VECT, "VirtualJVel", 10);

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
        if (!mIsVirtualUpdated) {
            mVirtualJPos = mRobot->getQ();
            mVirtualJVel.setZero();
            mIsVirtualUpdated = true;
        }
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
    cmd->qddot = qddot_des;
    //cmd->jtrq = mRobot->getMassMatrix() * qddot_des + mRobot->getGravity();
    cmd->jtrq = mRobot->getMassMatrix() * qddot_des + mRobot->getCoriolisGravity();
    if (mIsVirtualUpdated) {
        mVirtualJVel =
            myUtils::eulerIntegration(mVirtualJVel, cmd->qddot, SERVO_RATE);
        mVirtualJPos =
            myUtils::eulerIntegration(mVirtualJPos, cmd->qdot, SERVO_RATE);
    }

    Eigen::VectorXd nearly_zero = Eigen::VectorXd::Constant(10, 0.001);
    if (myUtils::isInBoundingBox(-nearly_zero, mRobot->getQdot(), nearly_zero) && mIsCollect) {
        std::cout << "data collect" << std::endl;
        _collectData();
        Eigen::VectorXd trq = Eigen::VectorXd::Zero(5);
        trq = cmd->jtrq.tail(5);
        torque_list.push_back(trq);
        mIsCollect = false;
        //std::cout << "22" << std::endl;
        //std::cout << trq[4] << std::endl;
        //std::cout << mRobot->getGravity()[9] << std::endl;
        //std::cout << trq[3] << std::endl;
    }

    if (!mIsSaved && mRobot->getTime() > 500) {
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

    mIsCollect = true;
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
    Eigen::MatrixXd T_j_j = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd T_j_com = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd R_w_com = Eigen::MatrixXd::Zero(3, 3);
    for (int i = 0; i < 4; ++i) {
        T_j_j = (mRobot->getBodyNodeIsometry(body_node_name[i]).inverse() *
            mRobot->getBodyNodeIsometry(body_node_name[i+1])).matrix();
        T_j_j_list[i].push_back(T_j_j);
    }
    for (int i = 0; i < 5; ++i) {
        T_j_com = (mRobot->getBodyNodeIsometry(body_node_name[i]).inverse() *
            mRobot->getBodyNodeCoMIsometry(body_node_name[i])).matrix();
        R_w_com = mRobot->getBodyNodeCoMIsometry(body_node_name[i]).linear();
        T_j_com_list[i].push_back(T_j_com);
        R_w_com_list[i].push_back(R_w_com);
    }
    // analytic
    //std::cout << "11" << std::endl;
/*    Eigen::VectorXd mg_w(6);*/
    //mg_w << 0, 0, 0, 0, 0, -0.4*9.81;
    //Eigen::MatrixXd R_w_com_aug = Eigen::MatrixXd::Zero(6, 6);
    //R_w_com_aug.block(0, 0, 3, 3) = R_w_com;
    /*R_w_com_aug.block(3, 3, 3, 3) = R_w_com;*/

    //Eigen::Isometry3d tmp = (mRobot->getBodyNodeIsometry(body_node_name[4]).inverse() *
            //mRobot->getBodyNodeCoMIsometry(body_node_name[4]));
    //std::cout << dart::math::getAdTMatrix(tmp.inverse()).transpose() * R_w_com_aug.transpose() * mg_w << std::endl;

    //Eigen::VectorXd mg4_w(6), mg3_w(6);
    //mg4_w<< 0, 0, 0, 0, 0, -0.4*9.81;
    //mg3_w << 0, 0, 0, 0, 0, -3.1*9.81;

    //Eigen::MatrixXd R_w_com4_aug = Eigen::MatrixXd::Zero(6, 6);
    //R_w_com4_aug.block(0, 0, 3, 3) = mRobot->getBodyNodeCoMIsometry(body_node_name[4]).linear();
    //R_w_com4_aug.block(3, 3, 3, 3) = mRobot->getBodyNodeCoMIsometry(body_node_name[4]).linear();

    //Eigen::MatrixXd R_w_com3_aug = Eigen::MatrixXd::Zero(6, 6);
    //R_w_com3_aug.block(0, 0, 3, 3) = mRobot->getBodyNodeCoMIsometry(body_node_name[3]).linear();
    //R_w_com3_aug.block(3, 3, 3, 3) = mRobot->getBodyNodeCoMIsometry(body_node_name[3]).linear();

    //Eigen::Isometry3d T34 = mRobot->getBodyNodeIsometry(body_node_name[3]).inverse() *
        //mRobot->getBodyNodeIsometry(body_node_name[4]);
    //Eigen::Isometry3d T3_com4 = T34 * (mRobot->getBodyNodeIsometry(body_node_name[4]).inverse() *
            //mRobot->getBodyNodeCoMIsometry(body_node_name[4]));
    //Eigen::Isometry3d T3_com3 = (mRobot->getBodyNodeIsometry(body_node_name[3]).inverse() *
            //mRobot->getBodyNodeCoMIsometry(body_node_name[3]));
    //std::cout << dart::math::getAdTMatrix(T3_com4.inverse()).transpose() * R_w_com4_aug.transpose() * mg4_w 
        //+ dart::math::getAdTMatrix(T3_com3.inverse()).transpose() * R_w_com3_aug.transpose() * mg3_w<< std::endl;

}

void AdmittanceTest::_dataSave() {
    std::cout << "start saving data" << std::endl;
    int num_data(T_j_j_list[0].size());
    try {

        YAML::Node sys_id_cfg;
        for (int i = 0; i < num_data; ++i) {
            for (int j = 0; j < 4; ++j) {
                sys_id_cfg["data"+std::to_string(i)][body_node_name[j]]["T_j_j"] =
                    T_j_j_list[j][i];
                sys_id_cfg["data"+std::to_string(i)][body_node_name[j]]["T_j_com"] =
                    T_j_com_list[j][i];
                sys_id_cfg["data"+std::to_string(i)][body_node_name[j]]["R_w_com"] =
                    R_w_com_list[j][i];
                sys_id_cfg["data"+std::to_string(i)][body_node_name[j]]["torque"] =
                    torque_list[i][j];
            }
            sys_id_cfg["data"+std::to_string(i)][body_node_name[4]]["T_j_com"] =
                T_j_com_list[4][i];
            sys_id_cfg["data"+std::to_string(i)][body_node_name[4]]["R_w_com"] =
                R_w_com_list[4][i];
            sys_id_cfg["data"+std::to_string(i)][body_node_name[4]]["torque"] =
                torque_list[i][4];
        }

        std::ofstream file_out(THIS_COM"Config/FixedDraco/SysID/MASS.yaml");
        file_out << sys_id_cfg;
    } catch (YAML::ParserException &e) { std::cout << e.what() << "\n"; }

    exit(0);
}
