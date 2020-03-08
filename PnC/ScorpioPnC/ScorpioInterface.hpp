#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"
#include "PnC/ScorpioPnC/ScorpioDefinition.hpp"

enum GRIPPER_STATUS {is_closing = 0,
                     is_holding = 1,
                    is_opening = 2,
                    idle = 3 };

class ScorpioStateProvider;

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
        gripper_cmd = GRIPPER_STATUS::idle;
    }
    virtual ~ScorpioCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
    GRIPPER_STATUS gripper_cmd;
};

class ScorpioInterface : public EnvInterface {
   protected:
    void _ParameterSetting();

    ScorpioStateProvider* sp_;

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

    bool IsReadyToMove();
    void MoveEndEffectorTo(double x, double y, double z);
    bool IsReadyToGrasp();
    void Grasp();
    bool IsReadyToRelease();
    void Release();
    void PrintPhase();
};
