#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class Scorpio2StateProvider;
class OSC;
class Task;

class Gripper2Ctrl : public Controller {
   public:
    Gripper2Ctrl(RobotSystem* _robot);
    virtual ~Gripper2Ctrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

   protected:
    Eigen::VectorXd ini_pos_q_;
    Eigen::VectorXd q_kp_;
    Eigen::VectorXd q_kd_;

    OSC* osc_;
    Task* joint_task_;

    void _build_constraint_matrix();
    void _build_active_joint_idx();
    void _task_setup();
    void _compute_torque(Eigen::VectorXd & gamma );

    std::vector<int> active_joint_idx_;
    std::vector<bool> active_joint_;
    Eigen::MatrixXd Jc_;
    Scorpio2StateProvider* sp_;
    double ctrl_start_time_;
};
