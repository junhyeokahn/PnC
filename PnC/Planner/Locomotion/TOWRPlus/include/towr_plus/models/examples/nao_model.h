#pragma once

#include <towr_plus/models/composite_rigid_body_dynamics.h>
#include <towr_plus/models/endeffector_mappings.h>
#include <towr_plus/models/examples/nao_composite_rigid_body_inertia.h>
#include <towr_plus/models/kinematic_model.h>

namespace towr_plus {

class NaoKinematicModel : public KinematicModel {
public:
  NaoKinematicModel() : KinematicModel(2) {
    const double x_nominal_b = -0.0;
    const double y_nominal_b = 0.05;
    const double z_nominal_b = -0.342;

    foot_half_length_ = 0.07;
    foot_half_width_ = 0.04;

    nominal_stance_.at(L) << x_nominal_b, y_nominal_b, z_nominal_b;
    nominal_stance_.at(R) << x_nominal_b, -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.03, 0.01, 0.04;
    min_dev_from_nominal_ << -0.03, -0.01, -0.0;
  }
};

class NaoDynamicModel : public CompositeRigidBodyDynamics {
public:
  NaoDynamicModel()
      : CompositeRigidBodyDynamics(
            5.3, 2, std::make_shared<NaoCompositeRigidBodyInertia>()) {}
};

} /* namespace towr_plus */
