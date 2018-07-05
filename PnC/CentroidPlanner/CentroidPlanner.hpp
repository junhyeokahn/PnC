#pragma once

#include "Planner.hpp"
#include "CentroidModel.hpp"
#include "DynamicsState.hpp"
#include "ContactPlanInterface.hpp"

enum class Heuristic { trustRegion, softConstraint, timeOptimization };

class CentroidPlannerParameter : public PlannerParameter
{
public:
    CentroidPlannerParameter (){};
    virtual ~CentroidPlannerParameter (){};

    void paramSetFromYaml(const std::string & configFile_);

    Heuristic heuristic;

    /*! helper string variables for the optimization problem */
    std::string cfgFile, saveDynamicsFile, defaultSolverSettingFile;

    /*! helper integer variables for the optimization problem */
    int numComViaPoints, numActEEfs, numTimeSteps, maxTimeIterations;

    /*! helper boolean variables for the optimization problem */
    bool isStoreData, isTimeHorizonFixed, isFrictionConeLinear, isDefaultSolverSetting;

    /*! helper double variables for the optimization problem */
    double gravity, timeStep, robotMass, timeHorizon, frictionCoeff, maxTimeResidualTolerance,
           minTimeResidualImprovement, massTimesGravity, wTrqArm, wTrqLeg, wTimePenalty, wTime;

    /*! helper vector variables for the optimization problem */
    Eigen::Vector2d timeRange, torqueRange;
    Eigen::Vector3d externalForce, comDisplacement;
    Eigen::Vector3d wCom, wLMom, wLMomD, wLMomFinal, wAMom, wAMomD, wAMomFinal,
        wComVia, wFrcArm, wFrcLeg, wDFrcArm, wDFrcLeg, gravityVector,
        wLMomTrack, wAMomTrack;

    /*! via points for center of mass motion */
    std::vector<Eigen::VectorXd> comViaPoints;

    /*! region of support for end-effectors */
    std::array<Eigen::VectorXd, CentroidModel::numEEf> copRange;

    /*! offset of end-effectors from center of mass */
    std::array<Eigen::VectorXd, CentroidModel::numEEf> eEfOffset;

    /*! maximum end-effector lengths for legs and arms */
    Eigen::Matrix<double, CentroidModel::numEEf, 1> maxEEfLengths;

    DynamicsState initialState;
    DynamicsStateSequence refDynamicsStateSequence;
    ContactPlanInterface contactPlanInterface;
};

class CentroidPlanner : public Planner
{
    public:
        CentroidPlanner ();
        virtual ~CentroidPlanner ();

    private:
        /* data */
        virtual void _doPlan();
        virtual void _evalTrajectory( double time,
                Eigen::VectorXd & pos,
                Eigen::VectorXd & vel,
                Eigen::VectorXd & trq);
};
