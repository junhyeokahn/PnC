#pragma once

#include <Utils/IO/IOUtilities.hpp>

class InterruptLogic {
 public:
  InterruptLogic() { resetFlags(); }
  virtual ~InterruptLogic() {}
  virtual void processInterrupts() { resetFlags(); }

  virtual void resetFlags() {
    b_interrupt_button_p = false;
    b_interrupt_button_r = false;

    b_interrupt_button_w = false;
    b_interrupt_button_a = false;
    b_interrupt_button_s = false;
    b_interrupt_button_d = false;
    b_interrupt_button_q = false;
    b_interrupt_button_e = false;

    b_interrupt_button_x = false;
    b_interrupt_button_j = false;
    b_interrupt_button_k = false;
  }

  bool b_interrupt_button_p;
  bool b_interrupt_button_r;
  bool b_interrupt_button_w;
  bool b_interrupt_button_a;
  bool b_interrupt_button_s;
  bool b_interrupt_button_d;
  bool b_interrupt_button_q;
  bool b_interrupt_button_e;

  bool b_interrupt_button_x;
  bool b_interrupt_button_j;
  bool b_interrupt_button_k;
};
