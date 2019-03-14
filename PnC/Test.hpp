#pragma once

#include <vector>

#include <Configuration.h>
#include <PnC/Controller.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

class Controller;
class RobotSystem;

class Test {
   public:
    Test(RobotSystem* _robot) : b_first_visit_(true) {
        DataManager::GetDataManager()->RegisterData(&phase_, INT, "phase");
        robot_ = _robot;
    };
    virtual ~Test(){};

    virtual void TestInitialization() = 0;
    void getCommand(void* _command) {
        AdditionalUpdate_();
        if (b_first_visit_) {
            state_list_[phase_]->firstVisit();
            b_first_visit_ = false;
        }
        state_list_[phase_]->oneStep(_command);
        if (state_list_[phase_]->endOfPhase()) {
            state_list_[phase_]->lastVisit();
            phase_ = _NextPhase(phase_);
            b_first_visit_ = true;
        }
    };

    int getPhase() { return phase_; }

   protected:
    virtual int _NextPhase(const int& phase) = 0;
    virtual void AdditionalUpdate_(){};

    bool b_first_visit_;
    int phase_;
    std::vector<Controller*> state_list_;

    RobotSystem* robot_;
};
