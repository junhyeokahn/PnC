#pragma once

#include <PnC/InterruptLogic.hpp>

// Forward Declare Control Architecture
class LaikagoControlArchitecture;
class LaikagoStateProvider;

class WalkingInterruptLogic : public InterruptLogic {
 public:
  WalkingInterruptLogic(LaikagoControlArchitecture* ctrl_arch_);
  ~WalkingInterruptLogic();

  void processInterrupts();

  LaikagoControlArchitecture* ctrl_arch_;
  LaikagoStateProvider* sp_;

  double swaying_dis_;
};
