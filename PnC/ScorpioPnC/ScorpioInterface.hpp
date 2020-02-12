#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"
#include "PnC/ScorpioPnC/ScorpioDefinition.hpp"

//class DracoStateProvider;
//class DracoStateEstimator;

class ScorpioSensorData {
   public:
    ScorpioSensorData() {
        q = Eigen::VectorXd::Zero(Scorpio::n_dof);
        qdot = Eigen::VectorXd::Zero(Scorpio::n_dof);
        //passive_q = Eigen::VectorXd::Zero(Scorpio::n_vdof);
        //passive_qdot = Eigen::VectorXd::Zero(Scorpio::n_vdof);
    }
    virtual ~ScorpioSensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    //Eigen::VectorXd passive_q;
    //Eigen::VectorXd passive_qdot;
};

class ScorpioCommand {
   public:
    ScorpioCommand() {
        q = Eigen::VectorXd::Zero(Scorpio::n_adof);
        qdot = Eigen::VectorXd::Zero(Scorpio::n_adof);
        jtrq = Eigen::VectorXd::Zero(Scorpio::n_adof);
    }
    virtual ~ScorpioCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class ScorpioInterface : public EnvInterface {
   protected:
    void _ParameterSetting();

    //DracoStateEstimator* state_estimator_;
    //DracoStateProvider* sp_;

    //void CropTorque_(ScorpioCommand*);
    bool Initialization_(ScorpioSensorData*, ScorpioCommand*);

    int count_;
    int waiting_count_;
    bool test_initialized;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;


   public:
    ScorpioInterface();
    virtual ~ScorpioInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);
};
