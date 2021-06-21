#pragma once

#include <towr_plus/models/composite_rigid_body_dynamics.h>
#include <towr_plus/models/endeffector_mappings.h>
#include <towr_plus/models/examples/valkyrie_composite_rigid_body_inertia.h>
#include <towr_plus/models/kinematic_model.h>

namespace towr_plus {

class ValkyrieKinematicModel : public KinematicModel {
public:
  ValkyrieKinematicModel() : KinematicModel(2) {
    const double x_nominal_b = 0.;
    const double y_nominal_b = 0.137;
    const double z_nominal_b = -0.98;

    foot_half_length_ = 0.135;
    foot_half_width_ = 0.08;

    nominal_stance_.at(L) << x_nominal_b, y_nominal_b, z_nominal_b;
    nominal_stance_.at(R) << x_nominal_b, -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.18, 0.1, 0.05;
    min_dev_from_nominal_ << -0.18, -0.1, -0.01;
  }
};

class ValkyrieDynamicModel : public CompositeRigidBodyDynamics {
public:
  ValkyrieDynamicModel()
      : CompositeRigidBodyDynamics(
            137.28, 2, std::make_shared<ValkyrieCompositeRigidBodyInertia>()) {}
};

} /* namespace towr_plus */
