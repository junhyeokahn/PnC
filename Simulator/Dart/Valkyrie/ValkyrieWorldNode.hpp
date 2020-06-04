#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

class EnvInterface;
class ValkyrieSensorData;
class ValkyrieCommand;

class ValkyrieWorldNode : public dart::gui::osg::WorldNode {
 private:
  void get_contact_switch_data_(bool& rfoot_contact, bool& lfoot_contact);
  void set_params_();
  void get_ft_data_();

  EnvInterface* interface_;
  ValkyrieSensorData* sensor_data_;
  ValkyrieCommand* command_;

  dart::simulation::WorldPtr world_;
  dart::dynamics::SkeletonPtr robot_;
  dart::dynamics::SkeletonPtr ground_;

  Eigen::VectorXd trq_cmd_;

  int count_;
  double t_;
  double servo_rate_;
  int n_dof_;
  double kp_;
  double kd_;
  Eigen::VectorXd trq_lb_;
  Eigen::VectorXd trq_ub_;

 public:
  ValkyrieWorldNode(const dart::simulation::WorldPtr& world);
  virtual ~ValkyrieWorldNode();

  void customPreStep() override;

  // User buttons
  bool b_button_p;
  bool b_button_r;
  bool b_button_w;
  bool b_button_a;
  bool b_button_s;
  bool b_button_d;
  bool b_button_q;
  bool b_button_e;
  bool b_button_x;
  bool b_button_j;
  bool b_button_k;

  void enable_button_p_fjlag() { b_button_p = true; }
  void enable_button_r_flag() { b_button_r = true; }
  void enable_button_w_flag() { b_button_w = true; }
  void enable_button_a_flag() { b_button_a = true; }
  void enable_button_s_flag() { b_button_s = true; }
  void enable_button_d_flag() { b_button_d = true; }
  void enable_button_q_flag() { b_button_q = true; }
  void enable_button_e_flag() { b_button_e = true; }
  void enable_button_x_flag() { b_button_x = true; }
  void enable_button_j_flag() { b_button_j = true; }
  void enable_button_k_flag() { b_button_k = true; }

  void reset_button_flags() {
    b_button_p = false;
    b_button_r = false;
    b_button_w = false;
    b_button_a = false;
    b_button_s = false;
    b_button_d = false;
    b_button_q = false;
    b_button_e = false;
    b_button_x = false;
    b_button_j = false;
    b_button_k = false;
  }
};
