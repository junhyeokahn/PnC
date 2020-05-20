#pragma once

#include <vector>

#include <Configuration.h>
#include <PnC/Controller.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

class Controller;
class RobotSystem;

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

    RobotSystem* robot_;
};
