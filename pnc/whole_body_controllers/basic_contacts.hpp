#pragma once

#include <pnc/whole_body_controllers/contact.hpp>

class RobotSystem;

class PointContact : public Contact {
public:
  PointContact(RobotSystem *robot, const std::string _target_id,
               const double _mu);
  virtual ~PointContact();

protected:
  void _update_jacobian();
  void _update_cone_constraint();
};

class SurfaceContact : public Contact {
public:
  SurfaceContact(RobotSystem *robot, const std::string _target_id,
                 const double _x, const double _y, const double _mu);
  virtual ~SurfaceContact();

protected:
  double x_;
  double y_;

  void _update_jacobian();
  void _update_cone_constraint();

  Eigen::MatrixXd _get_u(double _x, double _y, double _mu);
};
