#pragma once

#include <PnC/ControlArchitecture.hpp>

#include <PnC/WBC/Task.hpp>
#include <PnC/WBC/ContactSpec.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <PnC/ValkyriePnC/CtrlSet/ValkyrieMainController.hpp>
#include <PnC/ValkyriePnC/TaskAndForceContainers/ValkyrieTaskAndForceContainer.hpp>

#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>

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
    void _SettingParameter();

    YAML::Node cfg_;

    // Temporary -------------
    bool b_first_visit_;
    Controller* balance_ctrl_;
    // -----------------------

  void _InitializeParameters();

  public:
    // Task and Force Containers
    ValkyrieTaskAndForceContainer* taf_container_;
    // Controller Object
    ValkyrieMainController* main_controller_;
};
