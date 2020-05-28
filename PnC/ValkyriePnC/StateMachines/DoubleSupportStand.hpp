#pragma once

#include <PnC/StateMachine.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <Utils/Math/BSplineBasic.h>

// Forward Declare. Will need to be defined in cpp file.
class ValkyrieControlArchitecture;
class ValkyrieTaskAndForceContainer;
class ValkyrieMainController;

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

    void switchStateButtonTrigger(){
      state_switch_button_trigger_ = true;
    }

  protected:
    ValkyrieStateProvider* sp_;
    ValkyrieControlArchitecture* val_ctrl_arch_;
    ValkyrieTaskAndForceContainer* taf_container_;

    double ctrl_start_time_;
    double end_time_;
    double time_to_max_normal_force_;

    void _taskUpdate();

    // // Stand Trajectory
    Eigen::VectorXd ini_com_pos_;
    Eigen::VectorXd des_com_pos_;
    Eigen::VectorXd des_com_vel_;
    Eigen::VectorXd des_com_acc_;
    Eigen::VectorXd com_pos_target_;

    BS_Basic<3,3,1,2,2> com_traj_;
    Eigen::Quaternion<double> ini_pelvis_quat_;

    void _SetBspline(const Eigen::VectorXd st_pos,
                     const Eigen::VectorXd des_pos);
    void _GetBsplineTrajectory();

    bool state_switch_button_trigger_;

};