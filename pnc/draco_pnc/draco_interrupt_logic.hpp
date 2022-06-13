#pragma once

#include "pnc/interrupt_logic.hpp"
#include "pnc/draco_pnc/draco_control_architecture.hpp"

#include <muvt_core/optimizer/optimizer.h>

class DracoInterruptLogic : public InterruptLogic {
public:
  DracoInterruptLogic(DracoControlArchitecture *_ctrl_arch);
  ~DracoInterruptLogic();

  void processInterrupts();

  DracoControlArchitecture *ctrl_arch_;
};
