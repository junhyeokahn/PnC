#include "pnc/whole_body_controllers/internal_constraint.hpp"

class DracoRollingJointConstraint : public InternalConstraint {
public:
  DracoRollingJointConstraint(RobotSystem *_robot);
  virtual ~DracoRollingJointConstraint(){};

private:
  /* data */
  virtual void _update_jacobian(){};
};
