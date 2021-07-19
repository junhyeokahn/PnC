#pragma once

#include <pnc/whole_body_controllers/contact.hpp>

class RobotSystem;

/// class PointContact
class PointContact : public Contact {
public:
  /// \{ \name Constructor and Destructor
  PointContact(RobotSystem *robot, const std::string _target_id,
               const double _mu);

  virtual ~PointContact();
  /// \}

protected:
  void _update_jacobian();
  void _update_cone_constraint();
};

/// class SurfaceContact
class SurfaceContact : public Contact {
public:
  /// \{ \name Constructor and Destructor
  SurfaceContact(RobotSystem *robot, const std::string _target_id,
                 const double _x, const double _y, const double _mu);

  virtual ~SurfaceContact();
  /// \}

protected:
  /// Half size of the feet length
  double x_;

  /// Half size of the feet width
  double y_;

  void _update_jacobian();
  void _update_cone_constraint();

  /// Compute constraint matrix for the surface contact case.
  Eigen::MatrixXd _get_u(double _x, double _y, double _mu);
};
