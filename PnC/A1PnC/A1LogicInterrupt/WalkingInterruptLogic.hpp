#pragma once

#include <PnC/InterruptLogic.hpp>

// Forward Declare Control Architecture
class A1ControlArchitecture;
class A1StateProvider;

class WalkingInterruptLogic : public InterruptLogic {
 public:
  WalkingInterruptLogic(A1ControlArchitecture* ctrl_arch_);
  ~WalkingInterruptLogic();

  void processInterrupts();

  A1ControlArchitecture* ctrl_arch_;
  A1StateProvider* sp_;

  double swaying_dis_;
};
