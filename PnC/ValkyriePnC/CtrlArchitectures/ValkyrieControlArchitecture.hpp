#pragma once
#include <PnC/ControlArchitecture.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <PnC/ValkyriePnC/StateMachines/StateMachineSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TrajectoryManagers/TrajectoryManagerSet.hpp>

#include <PnC/ValkyriePnC/CtrlSet/ValkyrieMainController.hpp>
#include <PnC/ValkyriePnC/TaskAndForceContainers/ValkyrieTaskAndForceContainer.hpp>

// Add planner
#include <PnC/PlannerSet/DCMPlanner/DCMPlanner.hpp>

// Add footstep list container
#include <vector>
#include <PnC/PlannerSet/DCMPlanner/Footstep.hpp>

namespace VALKYRIE_STATES {
    constexpr int BALANCE = 0;
    constexpr int INITIAL_TRANSFER = 1;
};  

class ValkyrieControlArchitecture : public ControlArchitecture {
  public:
    ValkyrieControlArchitecture(RobotSystem* _robot);
    virtual ~ValkyrieControlArchitecture();
    virtual void ControlArchitectureInitialization();
    virtual void getCommand(void* _command);

  protected:
    ValkyrieStateProvider* sp_;
    YAML::Node cfg_;

    void _InitializeParameters();
    bool b_state_first_visit_;

  public:
    // Task and Force Containers
    ValkyrieTaskAndForceContainer* taf_container_;
    // Controller Object
    ValkyrieMainController* main_controller_;
    // Add Planner
    DCMPlanner* dcm_planner_;

    // Trajectory Managers
    FootSE3TrajectoryManager* rfoot_trajectory_manager_; 
    FootSE3TrajectoryManager* lfoot_trajectory_manager_; 
    DCMPlannerTrajectoryManager* dcm_trajectory_manger_;

    // Footstep list container
    std::vector<Footstep> footstep_list_;


};
