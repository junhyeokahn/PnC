#pragma once

#include <pnc/atlas_pnc/atlas_control_architecture.hpp>
#include <pnc/interrupt_logic.hpp>

class AtlasInterruptLogic : public InterruptLogic {
public:
  AtlasInterruptLogic(AtlasControlArchitecture *_ctrl_arch);
  ~AtlasInterruptLogic();

  void processInterrupts();

  AtlasControlArchitecture *ctrl_arch_;
};
