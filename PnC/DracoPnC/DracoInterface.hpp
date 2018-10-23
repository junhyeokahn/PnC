#pragma once

#include "PnC/Interface.hpp"

class DracoStateEstimator;
class DracoStateProvider;

class DracoSensorData
{
public:
    Eigen::VectorXd imu_ang_vel;
    Eigen::VectorXd imu_acc;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
    Eigen::VectorXd temperature;
    Eigen::VectorXd motor_current;
    Eigen::VectorXd bus_voltage;
    Eigen::VectorXd bus_current;
    Eigen::VectorXd rotor_inertia;
    bool rfoot_contact;
    bool lfoot_contact;
};

class DracoCommand
{
public:
    bool turn_off;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};


class DracoInterface: public Interface
{
protected:
    int waiting_count_;

    void _ParameterSetting();
    bool _Initialization(DracoSensorData* , DracoCommand* );
    bool _UpdateTestCommand(DracoCommand* test_cmd);
    void _SetStopCommand(DracoSensorData*, DracoCommand* cmd);
    void _CopyCommand(DracoCommand* cmd );

    DracoCommand* test_cmd_;
    DracoStateEstimator * state_estimator_;
    DracoStateProvider * sp_;

    Eigen::VectorXd cmd_jtrq_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd data_torque_;
    Eigen::VectorXd data_temperature_;
    Eigen::VectorXd data_motor_current_;

    // safety
    Eigen::VectorXd jpos_max_;
    Eigen::VectorXd jpos_min_;
    Eigen::VectorXd jvel_max_;
    Eigen::VectorXd jvel_min_;
    Eigen::VectorXd jtrq_max_;
    Eigen::VectorXd jtrq_min_;
    bool stop_test_;

public:
    DracoInterface();
    virtual ~DracoInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);
};
