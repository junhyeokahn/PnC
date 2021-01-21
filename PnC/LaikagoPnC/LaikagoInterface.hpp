#pragma once

#include <vector>
#include "PnC/EnvInterface.hpp"
#define DOF 12



class LaikagoStateEstimator;
class LaikagoStateProvider;

class LaikagoSensorData {
 public:
  LaikagoSensorData() {
    imu_ang_vel = Eigen::VectorXd::Zero(3);
    imu_acc = Eigen::VectorXd::Zero(3);
    imu_mag = Eigen::VectorXd::Zero(3);
    q = Eigen::VectorXd::Zero(DOF);
    qdot = Eigen::VectorXd::Zero(DOF);
    jtrq = Eigen::VectorXd::Zero(DOF);
    temperature = Eigen::VectorXd::Zero(DOF);
    motor_current = Eigen::VectorXd::Zero(DOF);
    bus_voltage = Eigen::VectorXd::Zero(DOF);
    bus_current = Eigen::VectorXd::Zero(DOF);
    rotor_inertia = Eigen::VectorXd::Zero(DOF);
    rf_wrench = Eigen::VectorXd::Zero(6);
    lf_wrench = Eigen::VectorXd::Zero(6);
    rfoot_contact = false;
    rfoot_contact = false;
  }
  virtual ~LaikagoSensorData() {}

  Eigen::VectorXd imu_ang_vel;
  Eigen::VectorXd imu_acc;
  Eigen::VectorXd imu_mag;
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd jtrq;
  Eigen::VectorXd temperature;
  Eigen::VectorXd motor_current;
  Eigen::VectorXd bus_voltage;
  Eigen::VectorXd bus_current;
  Eigen::VectorXd rotor_inertia;
  Eigen::VectorXd rf_wrench;
  Eigen::VectorXd lf_wrench;
  bool rfoot_contact;
  bool lfoot_contact;
};

class LaikagoCommand {
 public:
  LaikagoCommand() {
    q = Eigen::VectorXd::Zero(DOF);
    qdot = Eigen::VectorXd::Zero(DOF);
    jtrq = Eigen::VectorXd::Zero(DOF);
  }
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd jtrq;
};

class LaikagoInterface : public EnvInterface {
 protected:
  int waiting_count_;

  void _ParameterSetting();
  bool _Initialization(LaikagoSensorData*, LaikagoCommand*);
  bool _UpdateTestCommand(LaikagoCommand* test_cmd);
  void _SetStopCommand(LaikagoSensorData*, LaikagoCommand* cmd);
  void _CopyCommand(LaikagoCommand* cmd);

  LaikagoStateEstimator* state_estimator_;
  LaikagoStateProvider* sp_;

  Eigen::VectorXd cmd_jtrq_;
  Eigen::VectorXd cmd_jpos_;
  Eigen::VectorXd cmd_jvel_;
  Eigen::VectorXd data_torque_;
  Eigen::VectorXd data_temperature_;
  Eigen::VectorXd data_motor_current_;
  Eigen::VectorXd rfoot_ati_;
  Eigen::VectorXd lfoot_ati_;
  // Eigen::VectorXd imu_acc_;
  // Eigen::VectorXd imu_angvel_;

  // safety
  Eigen::VectorXd jpos_max_;
  Eigen::VectorXd jpos_min_;
  Eigen::VectorXd jvel_max_;
  Eigen::VectorXd jvel_min_;
  Eigen::VectorXd jtrq_max_;
  Eigen::VectorXd jtrq_min_;
  bool stop_test_;

 public:
  LaikagoInterface();
  LaikagoInterface(int mpi_idx, int env_idx);
  virtual ~LaikagoInterface();
  virtual void getCommand(void* _sensor_data, void* _command_data);
};
