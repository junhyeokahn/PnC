#pragma once

#include "Planner.hpp"
#include "Clock.hpp"
#include "RobotSystem.hpp"
#include "Configuration.h"
#include "ContactPlanInterface.hpp"

class InvKinPlannerParameter : public PlannerParameter
{
public:
    InvKinPlannerParameter () : PlannerParameter() {};
    virtual ~InvKinPlannerParameter () {};

    void paramSetFromYaml(const std::string & configFile_);
    std::string cfgFile, saveSolutionFiles, robotModelPath;
    std::shared_ptr<RobotSystem> robot;
    std::vector< Eigen::Vector3d > rDes;
    std::vector< Eigen::Vector3d > lDes;
    std::vector< Eigen::Vector3d > kDes;
    std::vector< double > t;
    ContactPlanInterface contactPlanInterface;
    std::vector< Eigen::Vector3d > rfPosDes;
    std::vector< Eigen::Vector3d > rfVelDes;
    std::vector< Eigen::Vector3d > lfPosDes;
    std::vector< Eigen::Vector3d > lfVelDes;

};

class InvKinPlanner : public Planner
{
public:
    InvKinPlanner ();
    virtual ~InvKinPlanner ();

private:
    virtual void _doPlan();
    virtual void _evalTrajectory( double time,
                                  Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel,
                                  Eigen::VectorXd & trq);

    std::shared_ptr<InvKinPlannerParameter> mInvKinParam;

    std::vector< Eigen::VectorXd > mQSol;
    std::vector< Eigen::VectorXd > mQdotSol;
};
