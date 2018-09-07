#pragma once

#include "CentroidModel.hpp"
#include <array>
#include "Planner.hpp"
#include "Clock.hpp"
#include "Configuration.h"
#include "ContactPlanInterface.hpp"
#include "BSplineBasic.h"

#include <drake/common/find_resource.h>
#include <drake/multibody/ik_options.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_constraint.h>
#include <drake/multibody/rigid_body_ik.h>
#include <drake/multibody/rigid_body_tree.h>
#include "drake/math/eigen_sparse_triplet.h"

class InvKinPlannerParameter : public PlannerParameter
{
public:
    InvKinPlannerParameter () : PlannerParameter() {
        swingHeight = 0.05;
    };
    virtual ~InvKinPlannerParameter () {};

    void paramSetFromYaml(const std::string & configFile_);
    double swingHeight;
    Eigen::VectorXd initQ; // Should be reset for replanning
    std::string cfgFile, saveSolutionFiles, robotModelPath;
    std::shared_ptr<RigidBodyTree<double>> robot;
    std::vector< Eigen::Vector3d > rDes;
    std::vector< Eigen::Vector3d > lDes;
    std::vector< Eigen::Vector3d > kDes;
    std::vector< double > t;
    ContactPlanInterface contactPlanInterface;
    std::array<std::vector<BS_Basic<3, 3, 1, 2, 2>>, CentroidModel::numEEf> eEfSpline;
    std::array< std::vector<Eigen::Vector3d>, CentroidModel::numEEf > eEfPosDes;
    std::array< std::vector<Eigen::Quaternion<double>>, CentroidModel::numEEf > eEfOriDes;
    std::array< std::vector<Eigen::Vector3d>, CentroidModel::numEEf > eEfVelDes; // TODO do i need ee velocity constraint?
    std::array< int, CentroidModel::numEEf > eEfIdx;

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
    void _solutionCheck( int time_id);

    std::shared_ptr<InvKinPlannerParameter> mInvKinParam;

    std::vector< Eigen::VectorXd > mQSol;
    std::vector< Eigen::VectorXd > mQdotSol;
};
