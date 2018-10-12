#pragma once

#include "PnC/Test.hpp"
#include "Utils/BSplineBasic.h"
#include "PnC/FixedDracoPnC/FixedDracoInterface.hpp"

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

    std::vector< Eigen::MatrixXd > aug_T_j_com_list;
    std::vector< Eigen::MatrixXd > aug_R_w_com_list;
};
