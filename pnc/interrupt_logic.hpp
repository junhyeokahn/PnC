#pragma once

#include "utils/util.hpp"

/// class InterruptLogic
class InterruptLogic {
public:
  /// \{ \name Constructor and Destructor
  InterruptLogic() { resetFlags(); }
  virtual ~InterruptLogic() {}
  /// \}

  /// Process user interrupts
  virtual void processInterrupts() { resetFlags(); }

  /// Reset the interrupt flags. This should be called at the end of the method
  /// processInterrupts.
  virtual void resetFlags() {
    b_interrupt_button_f = false;
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
    b_interrupt_button_h = false;
    b_interrupt_button_l = false;

    b_interrupt_button_z = false;
    b_interrupt_button_c = false;
  }

  bool b_interrupt_button_f;
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
  bool b_interrupt_button_h;
  bool b_interrupt_button_l;

  bool b_interrupt_button_z;
  bool b_interrupt_button_c;
};
