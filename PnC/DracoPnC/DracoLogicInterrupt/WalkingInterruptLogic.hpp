#pragma once

#include <PnC/InterruptLogic.hpp>

// Forward Declare Control Architecture
class DracoControlArchitecture;
class DracoStateProvider;

class WalkingInterruptLogic : public InterruptLogic {
 public:
  WalkingInterruptLogic(DracoControlArchitecture* ctrl_arch_);
  ~WalkingInterruptLogic();

  void processInterrupts();

  DracoControlArchitecture* ctrl_arch_;
  DracoStateProvider* sp_;
};
