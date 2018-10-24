#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class ContactSpec;
class WBDC;
class WBDC_ExtraData;


class JPosSwingCtrl: public Controller{
    public:
        JPosSwingCtrl(RobotSystem* _robot);
        virtual ~JPosSwingCtrl();

        virtual void oneStep(void* cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const std::string & setting_file_name);

        void setMovingTime(double time) { end_time_ = time; }
        void setPosture(const Eigen::VectorXd & set_jpos){
            set_jpos_ = set_jpos;
            b_jpos_set_ = true;
        }

        // For sinusoidal trajectory test
        void setAmplitude( const Eigen::VectorXd & amp){ amp_ = amp; }
        void setFrequency( const Eigen::VectorXd & freq){ freq_ = freq; }
        void setPhase( const Eigen::VectorXd & phase){ phase_ = phase; }

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

        bool b_jpos_set_;
        Eigen::VectorXd set_jpos_;

        // For sinusoidal trajectory test
        Eigen::VectorXd amp_;
        Eigen::VectorXd freq_;
        Eigen::VectorXd phase_;

        void _jpos_task_setup();
        void _fixed_body_contact_setup();
        void _jpos_ctrl_wbdc_rotor(Eigen::VectorXd & gamma);

        double ctrl_start_time_;
        DracoStateProvider* sp_;
};
