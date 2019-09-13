#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

class EnvInterface;
class DracoSensorData;
class DracoCommand;

class DracoWorldNode : public dart::gui::osg::WorldNode {
   private:
    EnvInterface* Interface_;
    DracoSensorData* SensorData_;
    DracoCommand* Command_;

    void GetContactSwitchData_(bool& rfoot_contact, bool& lfoot_contact);
    void SetParams_();
    void PlotMPCResult_();
    void GetForceTorqueData_();

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
    DracoWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~DracoWorldNode();

    void customPreStep() override;
};
