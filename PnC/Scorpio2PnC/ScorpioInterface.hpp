#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"
#include "PnC/Scorpio2PnC/ScorpioDefinition.hpp"

enum GRIPPER2_STATUS {is_closing2 = 0,
                     is_holding2 = 1,
                    is_opening2 = 2,
                    idle2 = 3 };

class Scorpio2StateProvider;

class Scorpio2SensorData {
   public:
    Scorpio2SensorData() {
        q = Eigen::VectorXd::Zero(Scorpio::n_dof);
        qdot = Eigen::VectorXd::Zero(Scorpio::n_dof);
        //passive_q = Eigen::VectorXd::Zero(Scorpio::n_vdof);
        //passive_qdot = Eigen::VectorXd::Zero(Scorpio::n_vdof);
    }
    virtual ~Scorpio2SensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    //Eigen::VectorXd passive_q;
    //Eigen::VectorXd passive_qdot;
};

class Scorpio2Command {
   public:
    Scorpio2Command() {
        q = Eigen::VectorXd::Zero(Scorpio::n_adof);
        qdot = Eigen::VectorXd::Zero(Scorpio::n_adof);
        jtrq = Eigen::VectorXd::Zero(Scorpio::n_adof);
        gripper_cmd = GRIPPER2_STATUS::idle2;
    }
    virtual ~Scorpio2Command() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
    GRIPPER2_STATUS gripper_cmd;
};

class Scorpio2Interface : public EnvInterface {
   protected:
    void _ParameterSetting();

    Scorpio2StateProvider* sp_;

    //void CropTorque_(Scorpio2Command*);
    bool Initialization_(Scorpio2SensorData*, Scorpio2Command*);

    int count_;
    int waiting_count_;
    bool test_initialized;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;


   public:
    Scorpio2Interface();
    virtual ~Scorpio2Interface();
    virtual void getCommand(void* _sensor_data, void* _command_data);

    bool IsReadyToMove();
    void MoveEndEffectorTo(double x, double y, double z);
    bool IsReadyToGrasp();
    void Grasp();
    bool IsReadyToRelease();
    void Release();
    void PrintPhase();
};
