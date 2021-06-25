#pragma once

#include <PnC/AtlasPnC/AtlasControlArchitecture.hpp>
#include <PnC/InterruptLogic.hpp>

class AtlasInterruptLogic : public InterruptLogic {
public:
  AtlasInterruptLogic(AtlasControlArchitecture *_ctrl_arch);
  ~AtlasInterruptLogic();

  void processInterrupts();

  AtlasControlArchitecture *ctrl_arch_;
};
