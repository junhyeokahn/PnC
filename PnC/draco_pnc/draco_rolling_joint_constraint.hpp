#include "PnC/WBC/InternalConstraint.hpp"

class DracoRollingJointConstraint : public InternalConstraint {
public:
  DracoRollingJointConstraint(RobotSystem *_robot);
  virtual ~DracoRollingJointConstraint(){};

private:
  /* data */
  virtual void _update_jacobian(){};
};
