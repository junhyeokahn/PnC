#include "pnc/whole_body_controllers/internal_constraint.hpp"

class FixedDracoRollingJointConstraint : public InternalConstraint {
public:
  FixedDracoRollingJointConstraint(RobotSystem *_robot);
  virtual ~FixedDracoRollingJointConstraint(){};

private:
  /* data */
  virtual void _update_jacobian(){};
};
