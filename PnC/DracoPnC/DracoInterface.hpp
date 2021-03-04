#pragma once

#include <vector>
#include "PnC/EnvInterface.hpp"

class DracoStateEstimator;
class DracoStateProvider;
class filter;

class DracoSensorData {
 public:
  DracoSensorData() {
    imu_ang_vel = Eigen::VectorXd::Zero(3);
    imu_acc = Eigen::VectorXd::Zero(3);
    imu_mag = Eigen::VectorXd::Zero(3);
    q = Eigen::VectorXd::Zero(10);
    qdot = Eigen::VectorXd::Zero(10);
    jtrq = Eigen::VectorXd::Zero(10);
    temperature = Eigen::VectorXd::Zero(10);
    motor_current = Eigen::VectorXd::Zero(10);
    bus_voltage = Eigen::VectorXd::Zero(10);
    bus_current = Eigen::VectorXd::Zero(10);
    rotor_inertia = Eigen::VectorXd::Zero(10);
    // rf_wrench = Eigen::VectorXd::Zero(6); 
    // lf_wrench = Eigen::VectorXd::Zero(6);

    rf_front_wrench = Eigen::VectorXd::Zero(6);
    rf_back_wrench = Eigen::VectorXd::Zero(6);
    lf_front_wrench = Eigen::VectorXd::Zero(6);
    lf_back_wrench = Eigen::VectorXd::Zero(6);
    
    rfoot_contact = false;
    lfoot_contact = false;
  }
  virtual ~DracoSensorData() {}

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
  // Eigen::VectorXd rf_wrench;
  // Eigen::VectorXd lf_wrench;
  Eigen::VectorXd rf_front_wrench;
  Eigen::VectorXd rf_back_wrench;
  Eigen::VectorXd lf_front_wrench;
  Eigen::VectorXd lf_back_wrench;
  bool rfoot_contact;
  bool lfoot_contact;
};

class DracoCommand {
 public:
  DracoCommand() {
    q = Eigen::VectorXd::Zero(10);
    qdot = Eigen::VectorXd::Zero(10);
    jtrq = Eigen::VectorXd::Zero(10);
    Fr_estimated = Eigen::VectorXd::Zero(6*4);
    Fr_ext = Eigen::VectorXd::Zero(6);

    K_p = Eigen::VectorXd::Zero(10);
    K_d = Eigen::VectorXd::Zero(10);
  }
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd jtrq;

  Eigen::VectorXd Fr_estimated;
  Eigen::VectorXd Fr_ext;

  Eigen::VectorXd K_p;
  Eigen::VectorXd K_d;
};

class DracoInterface : public EnvInterface {
 protected:
  int waiting_count_;

  void _ParameterSetting();
  bool _Initialization(DracoSensorData*, DracoCommand*);
  bool _UpdateTestCommand(DracoCommand* test_cmd);
  void _SetStopCommand(DracoSensorData*, DracoCommand* cmd);
  void _CopyCommand(DracoCommand* cmd);

  DracoStateEstimator* state_estimator_;
  DracoStateProvider* sp_;

  Eigen::VectorXd cmd_jtrq_;
  Eigen::VectorXd cmd_jpos_;
  Eigen::VectorXd cmd_jvel_;
  Eigen::VectorXd data_torque_;
  Eigen::VectorXd data_temperature_;
  Eigen::VectorXd data_motor_current_;
  Eigen::VectorXd rfoot_ati_;
  Eigen::VectorXd lfoot_ati_;
  Eigen::VectorXd rfoot_front_ati_;
  Eigen::VectorXd rfoot_back_ati_;
  Eigen::VectorXd lfoot_front_ati_;
  Eigen::VectorXd lfoot_back_ati_;

  Eigen::VectorXd rfoot_front_est_;
  Eigen::VectorXd rfoot_back_est_;
  Eigen::VectorXd lfoot_front_est_;
  Eigen::VectorXd lfoot_back_est_;

  Eigen::VectorXd Fr_ext_;

  Eigen::VectorXd K_p_;
  Eigen::VectorXd K_d_;

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

  filter* x_force_est_;
  filter* y_force_est_;
  filter* z_force_est_;

  filter* x1_force_est_;
  filter* y1_force_est_;
  filter* z1_force_est_;

  filter* x2_force_est_;
  filter* y2_force_est_;
  filter* z2_force_est_;

  filter* x3_force_est_;
  filter* y3_force_est_;
  filter* z3_force_est_;

 public:
  DracoInterface();
  DracoInterface(int mpi_idx, int env_idx);
  virtual ~DracoInterface();
  virtual void getCommand(void* _sensor_data, void* _command_data);
};
