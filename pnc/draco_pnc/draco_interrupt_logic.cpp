#include <pnc/draco_pnc/draco_interrupt_logic.hpp>
#include <pnc/draco_pnc/draco_state_machine/double_support_balance.hpp>

DracoInterruptLogic::DracoInterruptLogic(DracoControlArchitecture *_ctrl_arch)
    : InterruptLogic() {
  util::PrettyConstructor(1, "Draco Walking Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
}

DracoInterruptLogic::~DracoInterruptLogic() {}

// Process Interrupts here
void DracoInterruptLogic::processInterrupts() {
  if (b_interrupt_button_p) {
    std::cout << "[Walking Interrupt Logic] button P pressed" << std::endl;
  }

  if (b_interrupt_button_r) {
    std::cout << "[Walking Interrupt Logic] button R pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------      COM Swaying   ---------" << std::endl;
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_swaying_trigger = true;
    }
  }

  if (b_interrupt_button_w) {
    std::cout << "[Walking Interrupt Logic] button W pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      ctrl_arch_->dcm_tm->walkForward();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Walking Forward  ---------" << std::endl;
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_walking_trigger = true;
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }
  if (b_interrupt_button_a) {
    if (ctrl_arch_->state == draco_states::kBalance) {
      ctrl_arch_->dcm_tm->strafeLeft();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Strafing Left    ---------" << std::endl;
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_walking_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }
  if (b_interrupt_button_s) {
    std::cout << "[Walking Interrupt Logic] button S pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      ctrl_arch_->dcm_tm->walkBackward();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "--------- Walking Backwards  ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_walking_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_d) {
    std::cout << "[Walking Interrupt Logic] button D pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      ctrl_arch_->dcm_tm->strafeRight();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Strafing Right   ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_walking_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_q) {
    std::cout << "[Walking Interrupt Logic] button Q pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      ctrl_arch_->dcm_tm->turnLeft();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------     Turn Left      ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_walking_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_e) {
    std::cout << "[Walking Interrupt Logic] button E pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      ctrl_arch_->dcm_tm->turnRight();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------     Turn Right      ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_walking_trigger = true;

    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_x) {
    std::cout << "[Walking Interrupt Logic] button X pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      ctrl_arch_->dcm_tm->walkInPlace();
      std::cout << "---------                        ---------" << std::endl;
      std::cout << "---------     Walk In Place      ---------" << std::endl;

      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_walking_trigger = true;

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
