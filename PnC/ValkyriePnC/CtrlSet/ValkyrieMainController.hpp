#pragma once

#include <PnC/Controller.hpp>

#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/IHWBC/IHWBC_JointIntegrator.hpp>

#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>

#include <PnC/ValkyriePnC/TaskAndForceContainers/ValkyrieTaskAndForceContainer.hpp>

class ValkyrieMainController : public Controller {
  public:
    ValkyrieMainController(ValkyrieTaskAndForceContainer* _taf_container, RobotSystem* _robot);
    virtual ~ValkyrieMainController();

    void getCommand(void* _cmd);
    virtual void ctrlInitialization(const YAML::Node& node);

  protected:
    ValkyrieTaskAndForceContainer* taf_container_;
    ValkyrieStateProvider* sp_;

    //  Processing Step for first visit
    void firstVisit();
    bool b_first_visit_;

    // Redefine PreProcessing Command
    void _PreProcessing_Command();
    // -------------------------------------------------------
    // Controller Objects
    // -------------------------------------------------------
    IHWBC* ihwbc_;
    Eigen::VectorXd Fd_des_;
    Eigen::VectorXd tau_cmd_;
    Eigen::VectorXd qddot_cmd_;

    IHWBC_JointIntegrator* ihwbc_joint_integrator_;
    Eigen::VectorXd des_jacc_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;

    // -------------------------------------------------------
    // Parameters
    // -------------------------------------------------------   
    // IHWBC Controller parameters
    double ihwbc_dt_; // Joint integration time
    double w_contact_weight_; // contact weights hierarchy (relative to task weights)
    double lambda_qddot_; // Joint acceleration variable normalization
    double lambda_Fr_; //  Reaction Force normalization
    bool b_enable_torque_limits_; // Enable IHWBC torque limits

    // Joint Integrator parameters
    double vel_freq_cutoff_; // Hz
    double pos_freq_cutoff_; // Hz
    double max_pos_error_; // radians. After position integrator, deviation from current position

    // Parent Functions not used
    void oneStep(void* _cmd);
    void lastVisit();
    bool endOfPhase();
    void _PostProcessing_Command();
};