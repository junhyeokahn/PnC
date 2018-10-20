#pragma once

#include "PnC/Test.hpp"
#include "Utils/BSplineBasic.h"
#include "PnC/FixedDracoPnC/FixedDracoInterface.hpp"
#include <array>

class AdmittanceTest: public Test
{
public:
    AdmittanceTest (RobotSystem* robot_);
    virtual ~AdmittanceTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    Eigen::VectorXd mTestInitQ;
    BS_Basic<10, 3, 0, 2, 2> mSpline;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    Eigen::VectorXd mLb;
    Eigen::VectorXd mUb;
    double mTransTime;
    double mStayTime;
    double mTestInitTime;
    double mSplineInitTime;
    Eigen::VectorXd mStayPosition;

    void _updateSpline();
    void _dataSave();
    void _collectData();
    bool mDoUpdate;
    bool mIsSaved;
    bool mIsCollect;

    std::vector< std::string > body_node_name;
    std::array< std::vector<Eigen::MatrixXd>, 4> T_j_j_list;
    std::array< std::vector<Eigen::MatrixXd>, 5> T_j_com_list;
    std::array< std::vector<Eigen::MatrixXd>, 5> R_w_com_list;
    std::vector< Eigen::VectorXd > torque_list;

    bool mIsVirtualUpdated;
    Eigen::VectorXd mVirtualJPos;
    Eigen::VectorXd mVirtualJVel;
};
