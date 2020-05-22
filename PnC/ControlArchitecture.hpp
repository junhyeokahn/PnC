#pragma once

#include <map>

#include <Configuration.h>
#include <PnC/StateMachine.hpp>
#include <PnC/Controller.hpp>

#include <PnC/RobotSystem/RobotSystem.hpp>

#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

// Generic Control Architecture Object
class ControlArchitecture {
   public:
    ControlArchitecture(RobotSystem* _robot) {
        DataManager::GetDataManager()->RegisterData(&state_, INT, "phase");
        robot_ = _robot;
    };
    virtual ~ControlArchitecture(){};

    virtual void ControlArchitectureInitialization() = 0;
    virtual void getCommand(void* _command) {};

    int getState() { return state_; }

   protected:
    int state_;

    std::map<StateIdentifier, StateMachine*> state_machines_;
    RobotSystem* robot_;
};
