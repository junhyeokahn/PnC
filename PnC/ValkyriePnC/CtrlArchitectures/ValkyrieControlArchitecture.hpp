#pragma once

#include <PnC/ControlArchitecture.hpp>

#include <PnC/WBC/Task.hpp>
#include <PnC/WBC/ContactSpec.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/IHWBC/IHWBC_JointIntegrator.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>

namespace VALKYRIE_STATES {
    constexpr int BALANCE = 0;
};  

class ValkyrieControlArchitecture : public ControlArchitecture {
  public:
    ValkyrieControlArchitecture(RobotSystem*);
    virtual ~ValkyrieControlArchitecture();
    virtual void ControlArchitectureInitialization();
    virtual void getCommand(void* _command);

  protected:
    ValkyrieStateProvider* sp_;
    void _SettingParameter();

    YAML::Node cfg_;

    // Temporary -------------
    bool b_first_visit_;
    Controller* balance_ctrl_;
    // -----------------------

  void _InitializeParameters();
  void _InitializeController();
  void _InitializeTasks();
  void _InitializeContacts();

  void _DeleteController();
  void _DeleteTasks();
  void _DeleteContacts();

  public:
    // -------------------------------------------------------
    // Controller Objects
    // -------------------------------------------------------
    IHWBC* ihwbc_;
    Eigen::VectorXd tau_cmd_;
    Eigen::VectorXd qddot_cmd_;

    IHWBC_JointIntegrator* ihwbc_joint_integrator_;
    Eigen::VectorXd des_jacc_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    // -------------------------------------------------------
    // Task Member variables
    // -------------------------------------------------------
    Task* com_task_;
    Task* pelvis_ori_task_;
    Task* ang_momentum_task_;
    Task* upper_body_task_;
    std::vector<int> upper_body_joint_indices_; // list of upperbody joint names

    Task* rfoot_center_pos_task_;
    Task* lfoot_center_pos_task_;
    Task* rfoot_center_ori_task_;
    Task* lfoot_center_ori_task_;

    // -------------------------------------------------------
    // Contact Member variables
    // -------------------------------------------------------    
    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;
    int dim_contact_;
    double lfoot_max_z_;
    double rfoot_max_z_;

    // -------------------------------------------------------
    // Parameters
    // -------------------------------------------------------
    
    // IHWBC Controller parameters
    double ihwbc_dt_; // Joint integration time
    double w_contact_weight_; // contact weights hierarchy (relative to task weights)
    double lambda_qddot_; // Joint acceleration variable normalization
    double lambda_Fr_; //  Reaction Force normalization

    // Task Hierarchy Weights
    Eigen::VectorXd w_task_hierarchy_;
    double w_task_com_;
    double w_task_ang_mom_;
    double w_task_upper_body_;
    double w_task_pelvis_;
    double w_task_rfoot_;
    double w_task_lfoot_;

    // Task Gains
    Eigen::VectorXd kp_com_;
    Eigen::VectorXd kd_com_;
    Eigen::VectorXd kp_ang_mom_;
    Eigen::VectorXd kd_ang_mom_;
    Eigen::VectorXd kp_pelvis_;
    Eigen::VectorXd kd_pelvis_;
    Eigen::VectorXd kp_joint_;
    Eigen::VectorXd kd_joint_;
    Eigen::VectorXd kp_upper_body_joint_;
    Eigen::VectorXd kd_upper_body_joint_;
    Eigen::VectorXd kp_foot_;
    Eigen::VectorXd kd_foot_;

};
