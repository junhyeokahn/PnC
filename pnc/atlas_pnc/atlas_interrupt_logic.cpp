#include <pnc/atlas_pnc/atlas_interrupt_logic.hpp>
#include <pnc/atlas_pnc/atlas_state_machine/double_support_balance.hpp>

AtlasInterruptLogic::AtlasInterruptLogic(AtlasControlArchitecture *_ctrl_arch)
    : InterruptLogic() {
  util::PrettyConstructor(1, "AtlasInterruptLogic");
  ctrl_arch_ = _ctrl_arch;
}

AtlasInterruptLogic::~AtlasInterruptLogic() {}

// Process Interrupts here
void AtlasInterruptLogic::processInterrupts() {
  if (b_interrupt_button_p) {
    std::cout << "[Walking Interrupt Logic] button P pressed" << std::endl;
  }

  if (b_interrupt_button_r) {
    std::cout << "[Walking Interrupt Logic] button R pressed" << std::endl;
  }

  if (b_interrupt_button_w) {
    std::cout << "[Walking Interrupt Logic] button W pressed" << std::endl;
    if (ctrl_arch_->state == AtlasStates::Balance) {
      ctrl_arch_->dcm_tm->walkForward();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Walking Forward  ---------" << std::endl;
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[AtlasStates::Balance]))
          ->b_state_switch_trigger = true;
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }
  if (b_interrupt_button_a) {
    if (ctrl_arch_->state == AtlasStates::Balance) {
      ctrl_arch_->dcm_tm->strafeLeft();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Strafing Left    ---------" << std::endl;
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[AtlasStates::Balance]))
          ->b_state_switch_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }
  if (b_interrupt_button_s) {
    std::cout << "[Walking Interrupt Logic] button S pressed" << std::endl;
    if (ctrl_arch_->state == AtlasStates::Balance) {
      ctrl_arch_->dcm_tm->walkBackward();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "--------- Walking Backwards  ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[AtlasStates::Balance]))
          ->b_state_switch_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_d) {
    std::cout << "[Walking Interrupt Logic] button D pressed" << std::endl;
    if (ctrl_arch_->state == AtlasStates::Balance) {
      ctrl_arch_->dcm_tm->strafeRight();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Strafing Right   ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[AtlasStates::Balance]))
          ->b_state_switch_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_q) {
    std::cout << "[Walking Interrupt Logic] button Q pressed" << std::endl;
    if (ctrl_arch_->state == AtlasStates::Balance) {
      ctrl_arch_->dcm_tm->turnLeft();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------     Turn Left      ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[AtlasStates::Balance]))
          ->b_state_switch_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_e) {
    std::cout << "[Walking Interrupt Logic] button E pressed" << std::endl;
    if (ctrl_arch_->state == AtlasStates::Balance) {
      ctrl_arch_->dcm_tm->turnRight();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------     Turn Right      ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[AtlasStates::Balance]))
          ->b_state_switch_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_x) {
    std::cout << "[Walking Interrupt Logic] button X pressed" << std::endl;
    if (ctrl_arch_->state == AtlasStates::Balance) {
      ctrl_arch_->dcm_tm->walkInPlace();
      std::cout << "---------                        ---------" << std::endl;
      std::cout << "---------     Walk In Place      ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[AtlasStates::Balance]))
          ->b_state_switch_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_j) {
    std::cout << "[Walking Interrupt Logic] button J pressed" << std::endl;
  }

  if (b_interrupt_button_k) {
    std::cout << "[Walking Interrupt Logic] button K pressed" << std::endl;
  }

  resetFlags();
}
