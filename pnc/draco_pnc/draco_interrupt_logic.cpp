#include "pnc/draco_pnc/draco_state_machine/double_support_move.hpp"
#include "pnc/draco_pnc/draco_state_machine/foot_landing.hpp"
#include "pnc/draco_pnc/draco_state_machine/foot_lifting.hpp"
#include "pnc/draco_pnc/draco_state_machine/foot_swing.hpp"
#include <pnc/draco_pnc/draco_interrupt_logic.hpp>
#include <pnc/draco_pnc/draco_state_machine/double_support_balance.hpp>
#include <pnc/draco_pnc/draco_state_machine/single_support_lifting.hpp>

DracoInterruptLogic::DracoInterruptLogic(DracoControlArchitecture *_ctrl_arch)
    : InterruptLogic() {
  util::PrettyConstructor(1, "Draco Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
}

DracoInterruptLogic::~DracoInterruptLogic() {}

// Process Interrupts here
void DracoInterruptLogic::processInterrupts() {
  if (b_interrupt_button_f) {
    std::cout << "[Walking Interrupt Logic] button F pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      std::cout << "---------                       ---------" << std::endl;
      std::cout << "---------   COM Interpolation   ---------" << std::endl;
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_interpolation_trigger = true;
    }
  }

  if (b_interrupt_button_r) {
    std::cout << "[Walking Interrupt Logic] button R pressed" << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      std::cout << "---------                  ---------" << std::endl;
      std::cout << "---------    COM Swaying   ---------" << std::endl;
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
    std::cout << "[Static Balancing Interrupt Logic] button J pressed"
              << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kMoveCoMToLFoot) {
      (static_cast<DoubleSupportMove *>(
           ctrl_arch_->state_machines[draco_states::kMoveCoMToLFoot]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kMoveCoMToRFoot) {
      (static_cast<DoubleSupportMove *>(
           ctrl_arch_->state_machines[draco_states::kMoveCoMToRFoot]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kMoveCoMToCenter) {
      (static_cast<DoubleSupportMove *>(
           ctrl_arch_->state_machines[draco_states::kMoveCoMToCenter]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kLFootSingleSupportLifting) {
      static_cast<SingleSupportLifting *>(
          ctrl_arch_->state_machines[draco_states::kLFootSingleSupportLifting])
          ->b_static_balancing_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kRFootSingleSupportLifting) {
      static_cast<SingleSupportLifting *>(
          ctrl_arch_->state_machines[draco_states::kRFootSingleSupportLifting])
          ->b_static_balancing_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kLFootLanding) {
      (static_cast<FootLanding *>(
           ctrl_arch_->state_machines[draco_states::kLFootLanding]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kRFootLanding) {
      (static_cast<FootLanding *>(
           ctrl_arch_->state_machines[draco_states::kRFootLanding]))
          ->b_static_walking_trigger = true;
    } else {
      std::cout << "[Error] No Matching Interruption Method" << std::endl;
    }
  }

  if (b_interrupt_button_k) {
    std::cout << "[Static Walking Interrupt Logic] button K pressed"
              << std::endl;
    if (ctrl_arch_->state == draco_states::kBalance) {
      (static_cast<DoubleSupportBalance *>(
           ctrl_arch_->state_machines[draco_states::kBalance]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kMoveCoMToLFoot) {
      (static_cast<DoubleSupportMove *>(
           ctrl_arch_->state_machines[draco_states::kMoveCoMToLFoot]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kMoveCoMToRFoot) {
      (static_cast<DoubleSupportMove *>(
           ctrl_arch_->state_machines[draco_states::kMoveCoMToRFoot]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kMoveCoMToCenter) {
      (static_cast<DoubleSupportMove *>(
           ctrl_arch_->state_machines[draco_states::kMoveCoMToCenter]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kLFootLifting) {
      (static_cast<FootLifting *>(
           ctrl_arch_->state_machines[draco_states::kLFootLifting]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kRFootLifting) {
      (static_cast<FootLifting *>(
           ctrl_arch_->state_machines[draco_states::kRFootLifting]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kLFootSwingStatic) {
      (static_cast<FootSwing *>(
           ctrl_arch_->state_machines[draco_states::kLFootSwingStatic]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kRFootSwingStatic) {
      (static_cast<FootSwing *>(
           ctrl_arch_->state_machines[draco_states::kRFootSwingStatic]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kLFootLanding) {
      (static_cast<FootLanding *>(
           ctrl_arch_->state_machines[draco_states::kLFootLanding]))
          ->b_static_walking_trigger = true;
    } else if (ctrl_arch_->state == draco_states::kRFootLanding) {
      (static_cast<FootLanding *>(
           ctrl_arch_->state_machines[draco_states::kRFootLanding]))
          ->b_static_walking_trigger = true;
    } else {
      std::cout << "[Error] No Matching Interruption Method" << std::endl;
    }
  }
  resetFlags();
}
