#pragma once

#include "pnc/interrupt_logic.hpp"
#include "pnc/draco_pnc/draco_control_architecture.hpp"

class FixedDracoInterruptLogic : public InterruptLogic {
public:
  FixedDracoInterruptLogic(DracoControlArchitecture *_ctrl_arch);
  ~FixedDracoInterruptLogic();

  void processInterrupts();

  DracoControlArchitecture *ctrl_arch_;
};
