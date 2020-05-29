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
    std::cout << "current state = " << val_ctrl_arch_->getState() << std::endl;
  }

  if (b_interrupt_button_r){
    std::cout << "[Walking Interrupt Logic] button R pressed" << std::endl;
    if (val_ctrl_arch_->getState() == VALKYRIE_STATES::BALANCE){
      std::cout << " -- Trigger Walk --" << std::endl;

      val_ctrl_arch_->dcm_trajectory_manger_->resetStepIndex();
      val_ctrl_arch_->dcm_trajectory_manger_->footstep_list_.clear();     
      val_ctrl_arch_->dcm_trajectory_manger_->populateStepInPlace(4, RIGHT_ROBOT_SIDE);     

      static_cast<DoubleSupportBalance*>(val_ctrl_arch_->state_machines_[VALKYRIE_STATES::BALANCE])->switchStateButtonTrigger();
    }else{
      std::cout << " -- Command Ignored --" << std::endl;
    }

  }
  if (b_interrupt_button_w){
    std::cout << "[Walking Interrupt Logic] button W pressed" << std::endl;
  }
  if (b_interrupt_button_a){
    std::cout << "[Walking Interrupt Logic] button A pressed" << std::endl;
  }
  if (b_interrupt_button_s){
    std::cout << "[Walking Interrupt Logic] button S pressed" << std::endl;
  }
  if (b_interrupt_button_d){
    std::cout << "[Walking Interrupt Logic] button D pressed" << std::endl;
  }
  if (b_interrupt_button_q){
    std::cout << "[Walking Interrupt Logic] button Q pressed" << std::endl;
  }
  if (b_interrupt_button_e){
    std::cout << "[Walking Interrupt Logic] button E pressed" << std::endl;
  }

  resetFlags();
}