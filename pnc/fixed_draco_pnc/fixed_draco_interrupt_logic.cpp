#include "pnc/fixed_draco_pnc/fixed_draco_interrupt_logic.hpp"

#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/end_effector_hold.hpp"

FixedDracoInterruptLogic::FixedDracoInterruptLogic(
    FixedDracoControlArchitecture *_ctrl_arch)
    : InterruptLogic() {
  util::PrettyConstructor(1, "FixedDraco Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
}

FixedDracoInterruptLogic::~FixedDracoInterruptLogic() {}

// Process Interrupts here
void FixedDracoInterruptLogic::processInterrupts() {
  if (b_interrupt_button_p) {
    std::cout << "[Interrupt Logic] button p pressed" << std::endl;
  }

  if (b_interrupt_button_r) {
    std::cout << "[Interrupt Logic] button r pressed" << std::endl;
  }

  if (b_interrupt_button_w) {
    std::cout << "[Interrupt Logic] button w pressed" << std::endl;
    if (ctrl_arch_->state == fixed_draco_states::kHold) {
      std::cout << "---------                     ---------" << std::endl;
      std::cout << "---------   Swing Left Hand   ---------" << std::endl;
      static_cast<EndEffectorHold *>(
          ctrl_arch_->state_machines[fixed_draco_states::kHold])
          ->b_lh_swaying_trigger = true;
    }
  }

  if (b_interrupt_button_a) {
    std::cout << "[Interrupt Logic] button a pressed" << std::endl;
    if (ctrl_arch_->state == fixed_draco_states::kHold) {
      std::cout << "---------                     ---------" << std::endl;
      std::cout << "---------   Swing Left Foot   ---------" << std::endl;
      static_cast<EndEffectorHold *>(
          ctrl_arch_->state_machines[fixed_draco_states::kHold])
          ->b_lf_swaying_trigger = true;
    }
  }
  if (b_interrupt_button_s) {
    std::cout << "[Interrupt Logic] button s pressed" << std::endl;
    if (ctrl_arch_->state == fixed_draco_states::kHold) {
      std::cout << "---------                     ---------" << std::endl;
      std::cout << "---------   Swing Right Hand   ---------" << std::endl;
      static_cast<EndEffectorHold *>(
          ctrl_arch_->state_machines[fixed_draco_states::kHold])
          ->b_rh_swaying_trigger = true;
    }
  }

  if (b_interrupt_button_d) {
    std::cout << "[Interrupt Logic] button d pressed" << std::endl;
    std::cout << "---------                      ---------" << std::endl;
    std::cout << "---------   Swing Right Foot   ---------" << std::endl;
    static_cast<EndEffectorHold *>(
        ctrl_arch_->state_machines[fixed_draco_states::kHold])
        ->b_rf_swaying_trigger = true;
  }

  if (b_interrupt_button_q) {
    std::cout << "[Interrupt Logic] button q pressed" << std::endl;
  }

  if (b_interrupt_button_e) {
    std::cout << "[Interrupt Logic] button e pressed" << std::endl;
  }

  if (b_interrupt_button_x) {
    std::cout << "[Interrupt Logic] button x pressed" << std::endl;
  }

  if (b_interrupt_button_j) {
    std::cout << "[Interrupt Logic] button j pressed" << std::endl;
  }

  if (b_interrupt_button_k) {
    std::cout << "[Interrupt Logic] button k pressed" << std::endl;
  }

  resetFlags();
}
