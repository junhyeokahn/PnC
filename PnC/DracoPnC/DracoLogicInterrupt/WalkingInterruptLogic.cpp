#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoCtrlArchitecture.hpp>
#include <PnC/DracoPnC/DracoLogicInterrupt/WalkingInterruptLogic.hpp>

WalkingInterruptLogic::WalkingInterruptLogic(
    DracoControlArchitecture* _ctrl_arch)
    : InterruptLogic() {
  myUtils::pretty_constructor(1, "Draco Walking Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider(ctrl_arch_->robot_);
  swaying_dis_ = 0.05;
}

WalkingInterruptLogic::~WalkingInterruptLogic() {}

// Process Interrupts here
void WalkingInterruptLogic::processInterrupts() {
  if (b_interrupt_button_p) {
    std::cout << "[Walking Interrupt Logic] button P pressed" << std::endl;
    std::cout << "---------                          ---------" << std::endl;
    std::cout << "---------     Swing CoM     ---------" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      double amp(0.02);
      double freq(0.3);
      ctrl_arch_->floating_base_lifting_up_manager_->initializeCoMSinusoid(
          sp_->curr_time, amp, freq);
    } else {
      // Do Nothing
    }
  }

  if (b_interrupt_button_r) {
    std::cout << "[Walking Interrupt Logic] button R pressed" << std::endl;
  }

  if (b_interrupt_button_w) {
    std::cout << "[Walking Interrupt Logic] button W pressed" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->walkForward();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Walking Forward  ---------" << std::endl;
      static_cast<DoubleSupportBalance*>(
          ctrl_arch_->state_machines_[DRACO_STATES::BALANCE])
          ->switchStateButtonTrigger();
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }
  if (b_interrupt_button_a) {
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->strafeLeft();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Strafing Left    ---------" << std::endl;
      static_cast<DoubleSupportBalance*>(
          ctrl_arch_->state_machines_[DRACO_STATES::BALANCE])
          ->switchStateButtonTrigger();
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }
  if (b_interrupt_button_s) {
    std::cout << "[Walking Interrupt Logic] button S pressed" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->walkBackward();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "--------- Walking Backwards  ---------" << std::endl;
      static_cast<DoubleSupportBalance*>(
          ctrl_arch_->state_machines_[DRACO_STATES::BALANCE])
          ->switchStateButtonTrigger();
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_d) {
    std::cout << "[Walking Interrupt Logic] button D pressed" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->strafeRight();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------   Strafing Right   ---------" << std::endl;
      static_cast<DoubleSupportBalance*>(
          ctrl_arch_->state_machines_[DRACO_STATES::BALANCE])
          ->switchStateButtonTrigger();
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_q) {
    std::cout << "[Walking Interrupt Logic] button Q pressed" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->turnLeft();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------     Turn Left      ---------" << std::endl;
      static_cast<DoubleSupportBalance*>(
          ctrl_arch_->state_machines_[DRACO_STATES::BALANCE])
          ->switchStateButtonTrigger();
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_e) {
    std::cout << "[Walking Interrupt Logic] button E pressed" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->turnRight();
      std::cout << "---------                    ---------" << std::endl;
      std::cout << "---------     Turn Right      ---------" << std::endl;
      static_cast<DoubleSupportBalance*>(
          ctrl_arch_->state_machines_[DRACO_STATES::BALANCE])
          ->switchStateButtonTrigger();
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_x) {
    std::cout << "[Walking Interrupt Logic] button X pressed" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->walkInPlace();
      std::cout << "---------                        ---------" << std::endl;
      std::cout << "---------     Walk In Place      ---------" << std::endl;
      static_cast<DoubleSupportBalance*>(
          ctrl_arch_->state_machines_[DRACO_STATES::BALANCE])
          ->switchStateButtonTrigger();
    } else {
      // std::cout << "-- Command Ignored. Please Wait for Double Support
      // Balance" << std::endl;
    }
  }

  if (b_interrupt_button_k) {
    std::cout << "[Walking Interrupt Logic] button K pressed" << std::endl;
    std::cout << "---------                          ---------" << std::endl;
    std::cout << "---------     Raise CoM     ---------" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      Eigen::VectorXd dis = Eigen::VectorXd::Zero(3);
      dis[2] = swaying_dis_;
      ctrl_arch_->floating_base_lifting_up_manager_->initializeCoMSwaying(
          sp_->curr_time, 4.0, dis);
    } else {
      // Do Nothing
    }
  }

  if (b_interrupt_button_j) {
    std::cout << "[Walking Interrupt Logic] button J pressed" << std::endl;
    std::cout << "---------                          ---------" << std::endl;
    std::cout << "---------     Lowering CoM     ---------" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      Eigen::VectorXd dis = Eigen::VectorXd::Zero(3);
      dis[2] = -swaying_dis_;
      ctrl_arch_->floating_base_lifting_up_manager_->initializeCoMSwaying(
          sp_->curr_time, 4.0, dis);
    } else {
      // Do Nothing
    }
  }

  if (b_interrupt_button_h) {
    std::cout << "[Walking Interrupt Logic] button H pressed" << std::endl;
    std::cout << "---------                          ---------" << std::endl;
    std::cout << "---------     Move CoM Left     ---------" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      Eigen::VectorXd dis = Eigen::VectorXd::Zero(3);
      dis[1] = swaying_dis_;
      ctrl_arch_->floating_base_lifting_up_manager_->initializeCoMSwaying(
          sp_->curr_time, 4.0, dis);
    } else {
      // Do Nothing
    }
  }

  if (b_interrupt_button_l) {
    std::cout << "[Walking Interrupt Logic] button L pressed" << std::endl;
    std::cout << "---------                          ---------" << std::endl;
    std::cout << "---------     Move CoM Right     ---------" << std::endl;
    if (ctrl_arch_->getState() == DRACO_STATES::BALANCE) {
      Eigen::VectorXd dis = Eigen::VectorXd::Zero(3);
      dis[1] = -swaying_dis_;
      ctrl_arch_->floating_base_lifting_up_manager_->initializeCoMSwaying(
          sp_->curr_time, 4.0, dis);
    } else {
      // Do Nothing
    }
  }

  resetFlags();
}
