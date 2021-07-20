#pragma once

#include "pnc/interrupt_logic.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"

class FixedDracoInterruptLogic : public InterruptLogic {
public:
  FixedDracoInterruptLogic(FixedDracoControlArchitecture *_ctrl_arch);
  ~FixedDracoInterruptLogic();

  void processInterrupts();

  FixedDracoControlArchitecture *ctrl_arch_;
};
