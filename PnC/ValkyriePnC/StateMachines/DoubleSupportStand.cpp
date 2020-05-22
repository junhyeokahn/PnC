#include <PnC/ValkyriePnC/StateMachines/DoubleSupportStand.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

DoubleSupportStand::DoubleSupportStand(const StateIdentifier state_identifier_in, ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
                   StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Stand");
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
}

DoubleSupportStand::~DoubleSupportStand(){

}


void DoubleSupportStand::firstVisit(){
}

void DoubleSupportStand::oneStep(){  
}
void DoubleSupportStand::lastVisit(){  
}
bool DoubleSupportStand::endOfState(){  
} 

StateIdentifier DoubleSupportStand::getNextState(){
}


void DoubleSupportStand::initialization(const YAML::Node& node){
}
