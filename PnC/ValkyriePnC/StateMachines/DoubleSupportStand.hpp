#pragma once

#include <PnC/StateMachine.hpp>

// Forward Declare. Will need to be defined in cpp file.
class ValkyrieControlArchitecture;

class DoubleSupportStand : public StateMachine{
  public:
  	DoubleSupportStand(const StateIdentifier state_identifier_in, 
  					   ValkyrieControlArchitecture* _ctrl_arch, 
  					   RobotSystem* _robot);
  	~DoubleSupportStand();

    void oneStep(); 
    void firstVisit(); 
    void lastVisit(); 
    bool endOfState(); 
    void initialization(const YAML::Node& node); 
    StateIdentifier getNextState(); 

  protected:
    ValkyrieControlArchitecture* val_ctrl_arch_;
};