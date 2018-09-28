#pragma once

#include <array>

#include "drake/common/trajectories/piecewise_polynomial.h"

#include "RobotSystem/CentroidModel.hpp"
#include "PnC/Planner.hpp"
#include "Configuration.h"
#include "PnC/PlannerSet/CentroidPlanner/ContactPlanInterface.hpp"
#include "Utils/BSplineBasic.h"

class PrePlannedCentroidPlannerParameter : public PlannerParameter
{
public:
    PrePlannedCentroidPlannerParameter () : PlannerParameter() {};
    virtual ~PrePlannedCentroidPlannerParameter () {};

    std::string trajectoryFile;
    double swingHeight;

private:
    /* data */
};

class PrePlannedCentroidPlanner : public Planner
{
public:
    PrePlannedCentroidPlanner ();
    virtual ~PrePlannedCentroidPlanner ();

private:
    /* data */
    virtual void _doPlan();
    virtual void _evalTrajectory( double time,
                                  Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel,
                                  Eigen::VectorXd & trq);
    std::shared_ptr<PrePlannedCentroidPlannerParameter> mPreComputedParam;
    std::vector< Eigen::Vector3d > mRDes;
    std::vector< Eigen::Vector3d > mLDes;
    std::vector< Eigen::Vector3d > mKDes;
    std::vector< double > mT;
    ContactPlanInterface mContactPlanInterface;
    std::array<std::vector<BS_Basic<3, 3, 1, 2, 2>>, CentroidModel::numEEf> mEEfSpline;
    std::array< std::vector<Eigen::Vector3d>, CentroidModel::numEEf > mEEfPosDes;
    std::array< std::vector<Eigen::Quaternion<double>>, CentroidModel::numEEf > mEEfOriDes;
    std::array< std::vector<Eigen::Vector3d>, CentroidModel::numEEf > mEEfVelDes;
};
