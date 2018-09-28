#pragma once

#include "PnC/Test.hpp"
#include "PnC/PlannerSet/CentroidPlanner/PrePlannedCentroidPlanner.hpp"

class WBLC;
class WBLC_ExtraData;
class WBLCContact;
class Task;

class CentroidKinematicOptimizationTest: public Test
{
public:
    CentroidKinematicOptimizationTest (RobotSystem* robot_);
    virtual ~CentroidKinematicOptimizationTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    Eigen::VectorXd mTestInitQ;
    Eigen::VectorXd mTestInitCoMPos;
    double mTestInitTime;

    // Planner
    //Planner* mPlanner;
    std::unique_ptr<PrePlannedCentroidPlanner> mPlanner;
    std::shared_ptr<PrePlannedCentroidPlannerParameter> mPlanningParam;
    Eigen::VectorXd mNominalCentroidState;
    Eigen::VectorXd mNominalJointState;
    double mInterpolationDuration;
    std::string mPrePlannedFile;
    double mFootSwingHeight;

    // Controller
    WBLC* mWBLC;
    WBLC_ExtraData* mWBLCExtraData;
    Task* mCentroidTask;
    Task* mJointTask;
    WBLCContact* mRfContact;
    WBLCContact* mLfContact;
    std::vector<Task*> mTaskList;
    std::vector<WBLCContact*> mContactList;
    void _updateContact(); // update mContactList
    void _updateTask(); // update mTaskList
    void _WBLCpreProcess(); // Set dynamic properties and cost
    void _WBLCpostProcess(); // unset task and contact

    Eigen::VectorXd mCentroidTaskKp;
    Eigen::VectorXd mCentroidTaskKd;
    Eigen::VectorXd mJointTaskKp;
    Eigen::VectorXd mJointTaskKd;

    Eigen::VectorXd mCentroidPosDes;
    Eigen::VectorXd mCentroidVelDes;
    Eigen::VectorXd mCentroidAccDes;
    Eigen::VectorXd mJointPosDes;
    Eigen::VectorXd mJointVelDes;
    Eigen::VectorXd mJointAccDes;
};
