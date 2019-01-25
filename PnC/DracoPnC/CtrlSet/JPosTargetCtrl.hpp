#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class ContactSpec;
class WBDC;
class WBDC_ExtraData;

class JPosTargetCtrl: public Controller{
    public:
        JPosTargetCtrl(RobotSystem* _robot);
        virtual ~JPosTargetCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const YAML::Node& node);

        void setMovingTime(double time) { end_time_ = time; }
        void setTargetPosition(const Eigen::VectorXd & jpos);
    protected:
        double end_time_;

        Task* jpos_task_;
        ContactSpec* fixed_body_contact_;
        WBDC* wbdc_;
        WBDC_ExtraData* wbdc_data_;

        Eigen::VectorXd jpos_ini_;
        Eigen::VectorXd jpos_target_;
        Eigen::VectorXd des_jpos_;
        Eigen::VectorXd des_jvel_;

        void _jpos_task_setup();
        void _fixed_body_contact_setup();
        void _jpos_ctrl_wbdc(Eigen::VectorXd & gamma);

        DracoStateProvider* sp_;
        double ctrl_start_time_;
};
