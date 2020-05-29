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

    // User buttons
    bool b_button_p;
    bool b_button_r;
    bool b_button_w;
    bool b_button_a;
    bool b_button_s;
    bool b_button_d;
    bool b_button_q;
    bool b_button_e;
    bool b_button_x;
    bool b_button_j;
    bool b_button_k;

    void enableButtonPFlag(){ b_button_p = true; }
    void enableButtonRFlag(){ b_button_r = true; }
    void enableButtonWFlag(){ b_button_w = true; }
    void enableButtonAFlag(){ b_button_a = true; }
    void enableButtonSFlag(){ b_button_s = true; }
    void enableButtonDFlag(){ b_button_d = true; }
    void enableButtonQFlag(){ b_button_q = true; }
    void enableButtonEFlag(){ b_button_e = true; }
    void enableButtonXFlag(){ b_button_x = true; }
    void enableButtonJFlag(){ b_button_j = true; }
    void enableButtonKFlag(){ b_button_k = true; }

    void resetButtonFlags(){
        b_button_p = false;
        b_button_r = false;
        b_button_w = false;
        b_button_a = false;
        b_button_s = false;
        b_button_d = false;
        b_button_q = false;
        b_button_e = false;
        b_button_x = false;
        b_button_j = false;
        b_button_k = false;
    }

};
