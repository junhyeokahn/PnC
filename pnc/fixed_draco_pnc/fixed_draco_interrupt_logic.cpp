#include <pnc/fixed_draco_pnc/fixed_draco_interrupt_logic.hpp>
#include <pnc/draco_pnc/draco_state_machine/double_support_balance.hpp>

FixedDracoInterruptLogic::FixedDracoInterruptLogic(FixedDracoControlArchitecture *_ctrl_arch)
    : InterruptLogic() {
  util::PrettyConstructor(1, "FixedDraco Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
}

FixedDracoInterruptLogic::~FixedDracoInterruptLogic() {}

// Process Interrupts here
void FixedDracoInterruptLogic::processInterrupts() {
  //if (b_interrupt_button_p) {
    //std::cout << "[Walking Interrupt Logic] button P pressed" << std::endl;
  //}

  //if (b_interrupt_button_r) {
    //std::cout << "[Walking Interrupt Logic] button R pressed" << std::endl;
    //if (ctrl_arch_->state == fixed_draco_states::kBalance) {
      //std::cout << "---------                    ---------" << std::endl;
      //std::cout << "---------      COM Swaying   ---------" << std::endl;
      //(static_cast<DoubleSupportBalance *>(
           //ctrl_arch_->state_machines[fixed_draco_states::kBalance]))
          //->b_swaying_trigger = true;
    //}
  //}

  //if (b_interrupt_button_w) {
    //std::cout << "[Walking Interrupt Logic] button W pressed" << std::endl;
    //if (ctrl_arch_->state == fixed_draco_states::kBalance) {
      //ctrl_arch_->dcm_tm->walkForward();
      //std::cout << "---------                    ---------" << std::endl;
      //std::cout << "---------   Walking Forward  ---------" << std::endl;
      //(static_cast<DoubleSupportBalance *>(
           //ctrl_arch_->state_machines[fixed_draco_states::kBalance]))
          //->b_walking_trigger = true;
    //} else {

    //}
  //}
  //if (b_interrupt_button_a) {

    //} else {

    //}
  //}
  //if (b_interrupt_button_s) {

    //} else {

    //}
  //}

  //if (b_interrupt_button_d) {

    //} else {

    //}
  //}

  //if (b_interrupt_button_q) {

    //} else {

    //}
  //}

  //if (b_interrupt_button_e) {

    //} else {

    //}
  //}

  //if (b_interrupt_button_x) {

    //} else {

    //}
  //}

  //if (b_interrupt_button_j) {
    //std::cout << "[Interrupt Logic] button J pressed" << std::endl;
  //}

  //if (b_interrupt_button_k) {
    //std::cout << "[Interrupt Logic] button K pressed" << std::endl;
  //}

  //resetFlags();
}
