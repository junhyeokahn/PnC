#pragma once

#include <PnC/TrajectoryManagerBase.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>

#include <PnC/PlannerSet/DCMPlanner/Footstep.hpp>
#include <Utils/Math/hermite_curve_vec.hpp>
#include <Utils/Math/hermite_quaternion_curve.hpp>

// Object to manage common trajectory primitives

class FootSE3TrajectoryManager : public TrajectoryManagerBase {
  public:
  	FootSE3TrajectoryManager(Task* _foot_pos_task, Task* _foot_ori_task, RobotSystem* _robot);
  	~FootSE3TrajectoryManager();	
    void paramInitialization(const YAML::Node& node);

    // Use current pose to set the task.
    void useCurrent();

    Task* foot_pos_task_;
    Task* foot_ori_task_;
    int link_idx_;

    // Hermite Curve containers
    HermiteCurveVec pos_hermite_curve_;
    HermiteQuaternionCurve quat_hermite_curve_;
};