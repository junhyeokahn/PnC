#pragma once
#include <PnC/ControlArchitecture.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <PnC/ValkyriePnC/StateMachines/StateMachineSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>

#include <PnC/ValkyriePnC/CtrlSet/ValkyrieMainController.hpp>
#include <PnC/ValkyriePnC/TaskAndForceContainers/ValkyrieTaskAndForceContainer.hpp>

// Add planner
#include <PnC/PlannerSet/DCMPlanner/DCMPlanner.hpp>

namespace VALKYRIE_STATES {
    constexpr int BALANCE = 0;
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
};
