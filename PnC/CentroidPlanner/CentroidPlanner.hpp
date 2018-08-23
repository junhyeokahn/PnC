#pragma once

#include "Planner.hpp"
#include "CentroidModel.hpp"
#include "DynamicsState.hpp"
#include "ContactPlanInterface.hpp"
#include "Configuration.h"
#include "Clock.hpp"
#include "solver/interface/Solver.hpp"

using namespace solver;
using namespace myUtils;

/**
 * Helper class to define a linear approximation of a friction cone.
 */
struct FrictionCone
{
    typedef Eigen::Matrix< double, 4, 3> ConeMat;
    void getCone(double fcoeff, FrictionCone::ConeMat& cone_mat)
    {
        if (fcoeff <= 0.001) { fcoeff = 0.001; }
        Eigen::Vector3d fconevec = Eigen::Vector3d(0.5*sqrt(2.0), 0.5*sqrt(2.0), -fcoeff); fconevec.normalize();
        Eigen::Matrix3d fconemat = Eigen::Matrix3d::Identity();
        double angle = 2*M_PI/4;
        fconemat(0,0) = cos(angle);    fconemat(0,1) = -sin(angle);
        fconemat(1,0) = sin(angle);    fconemat(1,1) =  cos(angle);
        for (int i=0; i<4; i++) {
            cone_mat.row(i) = fconevec;
            fconevec = fconemat * fconevec;
        }
    }
};


enum class Heuristic { trustRegion, softConstraint, timeOptimization };

class CentroidPlannerParameter : public PlannerParameter
{
public:
    CentroidPlannerParameter () : PlannerParameter() {};
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
        virtual void _doPlan();
        virtual void _evalTrajectory( double time,
                                      Eigen::VectorXd & pos,
                                      Eigen::VectorXd & vel,
                                      Eigen::VectorXd & trq);

        std::shared_ptr<CentroidPlannerParameter> mCentParam;

        /**
         * class that stores all information about the problem, interfaces between high level definition
         * of the problem and its corresponding mathematical construction to solve it with a solver instance
         */
        solver::Model mModel;

        /*! helper variables to construct linear and quadratic expressions */
        solver::LinExpr mLinCons;
        solver::DCPQuadExpr mQuadObjective, mQuadCons;

        /*! exit code of the optimization problem */
        solver::ExitCode mExitCode;

        /**
         * c++ vector containing all problem variables defined by the user. Does not include extra
         * variables required to write the problem in standard conic form.
         */
        std::vector<solver::Var> mVars;

        /**
         * initial configuration of the robot, including center of mass position, linear and angular momenta,
         * end-effectors configurations: activation, position, orientation
         */
        DynamicsState mIniState;

        /*! simple helper class to build a linear approximation of a friction cone */
        FrictionCone mFrictionCone;
        FrictionCone::ConeMat mConeMatrix;

        /**
         * dynamics sequence of dynamic states. This is the main interface between the user
         * and the planner. The user can find in this variable all the optimization results
         */
        DynamicsStateSequence mDynStateSeq;

        /*! clock for timing purposes */
        Clock mTimer;

        /*! type for optimization variable: continuous 'C' or binary 'B' and type of heuristic */
        char mVariableType;

        /*! helper boolean variables for the optimization problem */
        bool mHasConverged;

        /*! helper integer variables for the optimization problem */
        int mSize, mNumVars;

        /*! helper double variables for the optimization problem */
        double mSolveTime, mConvergenceErr, mLastConvergenceErr;

        /*! helper vector variables for the optimization problem */
        Eigen::Vector3d mComPosGoal;

        /*! helper optimization variables for the optimization problem */
        solver::OptimizationVariable mDt, mCom, mLMom, mAMom, mLMomD, mAMomD;
        std::array<solver::OptimizationVariable, CentroidModel::numEEf> mFrcWorld, mTrqLocal, mCopLocal, mUbVar, mLbVar;

        /*! helper matrices and vectors for the optimization problem */
        solver::OptimizationVariable::OptVector mSolution;
        solver::OptimizationVariable::OptMatrix mMatLb, mMatUb, mMatGuess, mComGuess, mLMomGuess, mAMomGuess;

        /**
         * function to initialize and configure the optimization
         * @param[in]  param_                     planning parameter
         */
        void _initialize();

        /**
         * function to add each variable to the Model and assign a unique identifier
         * to it, to be used by the optimizer to construct the problem
         * @param[in]  opt_var                      helper optimization variable for model predictive control
         * @param[in]  model                        instance of solver interface to collect constraints, objective and solve problem
         * @param[in]  vars                         vector of optimization variables
         */
        void _addVariableToModel(const OptimizationVariable& opt_var,
                Model& model,
                std::vector<Var>& vars);

        /**
         * function to parse equations and objective into optimization problem and attempt to find a solution
         * @param[in]  ref_sequence                 dynamics sequence to be used as momentum tracking reference (can be zeros).
         * @param[in]  update_tracking_objective    changes weights from regulation to tracking of momentum in ref_sequence
         * @return     ExitCode                     flag that indicates the optimization result (for example: optimal, infeasible)
         */
        solver::ExitCode _optimize(bool update_tracking_objective = false);

        /**
         * function to initialize optimization variables: type [continuous or binary],
         * guess value [if any], upper and lower bounds for the variable
         */
        void _initializeOptimizationVariables();

        void _updateTrackingObjective();

        /**
         * functions to update tracking objective for momentum from penalty to tracking
         * and attempt to find a solution to the optimization problem
         * @param[in]  ref_sequence                 dynamics sequence to be used as momentum tracking reference (can be zeros).
         * @param[in]  is_first_time                flag to indicate if this is the first time the solution is being constructed
         */
        void _internalOptimize(bool is_first_time = false);

        /**
         * functions that transfers optimal solution from model to helper class OptimizationVariable,
         * from helper class OptimizationVariable to dynamics sequence to be accessed by the user, and
         * from dynamics sequence to a file.
         * @param[in]  opt_var                      helper optimization variable for model predictive control
         * @param[in]  ref_sequence                 dynamics sequence to be used as momentum tracking reference (can be zeros).
         */
        void _saveSolution(solver::OptimizationVariable& opt_var);
        void _storeSolution();
        void _saveToFile(const DynamicsStateSequence& ref_sequence);
};


