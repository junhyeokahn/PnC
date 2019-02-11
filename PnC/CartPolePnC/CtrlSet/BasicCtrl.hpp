#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class BasicCtrl: public Controller{
    public:
        BasicCtrl(RobotSystem* _robot);
        virtual ~BasicCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const YAML::Node& node);

        void setDuration(double time) { duration_ = time; }
    protected:
        double duration_;
        int ctrl_count_;
};
