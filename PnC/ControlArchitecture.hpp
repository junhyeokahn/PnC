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
        DataManager::GetDataManager()->RegisterData(&phase_, INT, "phase");
        robot_ = _robot;
    };
    virtual ~ControlArchitecture(){};

    virtual void ControlArchitectureInitialization() = 0;
    virtual void getCommand(void* _command) {};

    int getPhase() { return phase_; }

   protected:
    int phase_;

    RobotSystem* robot_;
};
