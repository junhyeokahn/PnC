#pragma once

#include <PnC/StateMachine.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <Utils/Math/BSplineBasic.h>

// Forward Declare. Will need to be defined in cpp file.
class ValkyrieControlArchitecture;
class ValkyrieTaskAndForceContainer;
class ValkyrieMainController;

class SwingControl : public StateMachine{
  public:
    SwingControl(const StateIdentifier state_identifier_in, 
                   const int _leg_side,
                   ValkyrieControlArchitecture* _ctrl_arch, 
                   RobotSystem* _robot);
    ~SwingControl();

    void oneStep(); 
    void firstVisit(); 
    void lastVisit(); 
    bool endOfState(); 
    void initialization(const YAML::Node& node); 
    StateIdentifier getNextState(); 

  protected:
    ValkyrieStateProvider* sp_;
    ValkyrieControlArchitecture* val_ctrl_arch_;
    ValkyrieTaskAndForceContainer* taf_container_;

    int leg_side_;
    bool final_step_;

    double ctrl_start_time_;
    double end_time_;

    void _taskUpdate();
};