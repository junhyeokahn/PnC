#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

class EnvInterface;
class ValkyrieSensorData;
class ValkyrieCommand;

class ValkyrieWorldNode : public dart::gui::osg::WorldNode {
   private:
    void GetContactSwitchData_(bool& rfoot_contact, bool& lfoot_contact);
    void SetParams_();
    void PlotMPCResult_();
    void GetForceTorqueData_();

    EnvInterface* interface_;
    ValkyrieSensorData* sensor_data_;
    ValkyrieCommand* command_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr robot_;
    dart::dynamics::SkeletonPtr ground_;

    Eigen::VectorXd trq_cmd_;

    int count_;
    double t_;
    double servo_rate_;
    int n_dof_;
    double kp_;
    double kd_;
    Eigen::VectorXd trq_lb_;
    Eigen::VectorXd trq_ub_;

    bool b_plot_mpc_result_;

   public:
    ValkyrieWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~ValkyrieWorldNode();

    void customPreStep() override;

    bool b_button_p;
    bool b_button_r;
    void enableButtonPFlag(){
        b_button_p = true;
    }
    void enableButtonRFlag(){
        b_button_r = true;
    }
    void resetButtonFlags(){
        b_button_p = false;
        b_button_r = false;
    }

};
