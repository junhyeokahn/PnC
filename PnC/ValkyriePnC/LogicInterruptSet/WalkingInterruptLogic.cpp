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
    std::cout << "[Walking Interrupt Logic]button p pressed" << std::endl;
    std::cout << "current state = " << val_ctrl_arch_->getState() << std::endl;
  }
  if (b_interrupt_button_r){
    std::cout << "[Walking Interrupt Logic]button r pressed" << std::endl;
  }
  if (b_interrupt_button_w){
    std::cout << "[Walking Interrupt Logic]button w pressed" << std::endl;
  }
  if (b_interrupt_button_a){
    std::cout << "[Walking Interrupt Logic]button a pressed" << std::endl;
  }
  if (b_interrupt_button_s){
    std::cout << "[Walking Interrupt Logic]button s pressed" << std::endl;
  }
  if (b_interrupt_button_d){
    std::cout << "[Walking Interrupt Logic]button d pressed" << std::endl;
  }
  if (b_interrupt_button_q){
    std::cout << "[Walking Interrupt Logic]button q pressed" << std::endl;
  }
  if (b_interrupt_button_e){
    std::cout << "[Walking Interrupt Logic]button e pressed" << std::endl;
  }

  resetFlags();
}