#pragma once

#include <PnC/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <PnC/ConvexMPC/ConvexMPC.hpp>
#include <PnC/ConvexMPC/GaitScheduler.hpp>
#include <Eigen/Dense>


// Object to manage common trajectory primitives
class FloatingBaseTrajectoryManager : public TrajectoryManagerBase {
 public:
  FloatingBaseTrajectoryManager(Task* _com_task, Task* _base_ori_task,
                                RobotSystem* _robot, ConvexMPC* _mpc_planner,
                                GaitScheduler* _gait_scheduler);
  ~FloatingBaseTrajectoryManager(){};

  void initializeFloatingBaseTrajectory(const double _start_time,
                                        const Eigen::VectorXd& _target_com_pos);
  void initializeCoMSwaying(double _start_time, double _duration,
                            Eigen::VectorXd _dis);
  void initializeCoMSinusoid(double _start_time, double _amp, double _freq);
  void updateFloatingBaseDesired(const double current_time);
  void paramInitialization(const YAML::Node& node){};

  void solveMPC(bool b_fl_contact, bool b_fr_contact,
                bool b_rl_contact, bool b_rr_contact,
                Eigen::VectorXd com_vel_des,
                double target_height,
                Eigen::VectorXd& rxn_forces,
                double current_time);

  double getSwingTime();

  Task* com_task_;
  Task* base_ori_task_;

  Eigen::VectorXd com_pos_des_;
  Eigen::VectorXd com_vel_des_;
  Eigen::VectorXd com_acc_des_;

  // Inputs to MPC ComputeContactForces
  Eigen::VectorXd mpc_pos_des_;
  Eigen::VectorXd mpc_vel_des_;
  Eigen::VectorXd mpc_rpy_des_;
  Eigen::VectorXd mpc_rpydot_des_;
  Eigen::VectorXd foot_contact_states;
  Eigen::VectorXd foot_pos_body_frame;
  Eigen::VectorXd foot_friction_coeffs;

  Eigen::Vector3d frfoot_body_frame, flfoot_body_frame,
                  rrfoot_body_frame, rlfoot_body_frame;

  Eigen::Quaternion<double> base_ori_quat_des_;
  Eigen::VectorXd base_ori_des_;
  Eigen::VectorXd base_ang_vel_des_;
  Eigen::VectorXd base_ang_acc_des_;

  ConvexMPC* mpc_planner_;
  GaitScheduler* gait_scheduler_;

  // Updates the task desired values
  void updateDesired();

  double start_time_;
  double duration_;
  Eigen::VectorXd ini_com_pos_;
  Eigen::VectorXd ini_base_quat_;
  Eigen::VectorXd target_com_pos_;
  int base_id_;
  bool is_swaying;
  bool is_sinusoid;
  Eigen::VectorXd amp;
  Eigen::VectorXd freq;
  Eigen::VectorXd mid_point;

  private:
  Eigen::Vector3d toRPY(Eigen::Quaternion<double> quat);
};
