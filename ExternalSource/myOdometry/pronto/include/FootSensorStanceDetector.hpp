#pragma once
#include "ExternalSource/myOdometry/pronto/include/StanceEstimatorBase.hpp"

namespace pronto {
namespace quadruped {

class FootSensorStanceDetector : public StanceEstimatorBase {
public:

  bool getStance(LegBoolMap& stance) override;

  bool getGRF(LegVectorMap& grf) override;

  void setStance(const LegBoolMap& stance) override;

  bool isStance(LegID leg) const override;

private:
  LegBoolMap stance_;
};

}  // namespace quadruped
}  // namespace pronto
