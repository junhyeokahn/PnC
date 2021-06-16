#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

class EnvInterface;
class A1SensorData;
class A1Command;

class A1WorldNode : public dart::gui::osg::WorldNode {
 private:
  EnvInterface* interface_;
  A1SensorData* sensor_data_;
  A1Command* command_;

  void get_imu_data_(Eigen::VectorXd& ang_vel, Eigen::VectorXd& acc);
  void check_foot_contact_by_pos_(bool& frfoot_contact, bool& flfoot_contact, bool& rrfoot_contact, bool& rlfoot_contact );
  void set_parameters_();

  dart::dynamics::SkeletonPtr skel_;
  dart::dynamics::SkeletonPtr ground_;
  Eigen::VectorXd trq_cmd_;
  Eigen::VectorXd pos_cmd_;
  Eigen::VectorXd vel_cmd_;
  int dof_;
  double release_time_;
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd trq_lb_;
  Eigen::VectorXd trq_ub_;
  int count_;
  double t_;
  double servo_rate_;
  dart::simulation::WorldPtr world_;

  int tmp_counter;

 public:
  Eigen::VectorXd initial_jpos;

  A1WorldNode(const dart::simulation::WorldPtr& world);
  virtual ~A1WorldNode();
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
  bool b_button_h;
  bool b_button_l;

  void enableButtonPFlag() { b_button_p = true; }
  void enableButtonRFlag() { b_button_r = true; }
  void enableButtonWFlag() { b_button_w = true; }
  void enableButtonAFlag() { b_button_a = true; }
  void enableButtonSFlag() { b_button_s = true; }
  void enableButtonDFlag() { b_button_d = true; }
  void enableButtonQFlag() { b_button_q = true; }
  void enableButtonEFlag() { b_button_e = true; }
  void enableButtonXFlag() { b_button_x = true; }
  void enableButtonJFlag() { b_button_j = true; }
  void enableButtonKFlag() { b_button_k = true; }
  void enableButtonHFlag() { b_button_h = true; }
  void enableButtonLFlag() { b_button_l = true; }

  void resetButtonFlags() {
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
    b_button_h = false;
    b_button_l = false;
  }
};
