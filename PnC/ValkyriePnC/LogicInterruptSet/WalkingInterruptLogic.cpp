#include <PnC/ValkyriePnC/LogicInterruptSet/WalkingInterruptLogic.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

WalkingInterruptLogic::WalkingInterruptLogic(ValkyrieControlArchitecture* _val_ctrl_arch):
  InterruptLogic() {
  myUtils::pretty_constructor(1, "Valkyrie Walking Interrupt Logic");
  val_ctrl_arch_ = _val_ctrl_arch;
}

WalkingInterruptLogic::~WalkingInterruptLogic(){
}

// Process Interrupts here
void WalkingInterruptLogic::processInterrupts(){
  if (b_interrupt_button_p){
    std::cout << "[Walking Interrupt Logic] button P pressed" << std::endl;
  }

  if (b_interrupt_button_r){
    std::cout << "[Walking Interrupt Logic] button R pressed" << std::endl;
  }

  if (b_interrupt_button_w){
    std::cout << "[Walking Interrupt Logic] button W pressed" << std::endl;
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      val_ctrl_arch_->dcm_trajectory_manger_->walkForward();
      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << "-- Command Ignored. Please Wait for Double Support Balance" << std::endl;        
    }
  }
  if (b_interrupt_button_a){
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      val_ctrl_arch_->dcm_trajectory_manger_->strafeLeft();
      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << "-- Command Ignored. Please Wait for Double Support Balance" << std::endl;        
    }
  }
  if (b_interrupt_button_s){
    std::cout << "[Walking Interrupt Logic] button S pressed" << std::endl;
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      val_ctrl_arch_->dcm_trajectory_manger_->walkBackward();
      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << "-- Command Ignored. Please Wait for Double Support Balance" << std::endl;        
    }
  }

  if (b_interrupt_button_d){
    std::cout << "[Walking Interrupt Logic] button D pressed" << std::endl;
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      val_ctrl_arch_->dcm_trajectory_manger_->strafeRight();
      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << "-- Command Ignored. Please Wait for Double Support Balance" << std::endl;        
    }
  }

  if (b_interrupt_button_q){
    std::cout << "[Walking Interrupt Logic] button Q pressed" << std::endl;
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      val_ctrl_arch_->dcm_trajectory_manger_->turnLeft();
      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << "-- Command Ignored. Please Wait for Double Support Balance" << std::endl;        
    }
  }

  if (b_interrupt_button_e){
    std::cout << "[Walking Interrupt Logic] button E pressed" << std::endl;
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      val_ctrl_arch_->dcm_trajectory_manger_->turnRight();
      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << "-- Command Ignored. Please Wait for Double Support Balance" << std::endl;        
    }
  }

  if (b_interrupt_button_x){
    std::cout << "[Walking Interrupt Logic] button X pressed" << std::endl;
    std::cout << "[Walking Interrupt Logic] button E pressed" << std::endl;
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      val_ctrl_arch_->dcm_trajectory_manger_->walkInPlace();
      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << "-- Command Ignored. Please Wait for Double Support Balance" << std::endl;        
    }

  }

  if (b_interrupt_button_j){
    std::cout << "[Walking Interrupt Logic] button J pressed" << std::endl;
    val_ctrl_arch_->upper_body_joint_trajectory_manager_->initializeLowerRightArmNow();
  }

  if (b_interrupt_button_k){
    std::cout << "[Walking Interrupt Logic] button K pressed" << std::endl;
    val_ctrl_arch_->upper_body_joint_trajectory_manager_->initializeRaiseRightArmNow();

  }

  resetFlags();
}