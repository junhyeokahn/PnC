#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

class EnvInterface;
class AtlasSensorData;
class AtlasCommand;

class AtlasWorldNode : public dart::gui::osg::WorldNode {
   private:
    void HoldXY_();
    void GetImuData_(Eigen::VectorXd& ang_vel, Eigen::VectorXd& acc);
    void GetContactSwitchData_(bool& rfoot_contact, bool& lfoot_contact);
    void ManipulateCameraPos_();
    void PlotTargetLocation_();

    EnvInterface* interface_;
    AtlasSensorData* sensor_data_;
    AtlasCommand* command_;

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
    bool b_show_viewer_;
    bool b_manipulate_camera_;
    bool b_show_target_frame_;

    Eigen::VectorXd trq_lb_;
    Eigen::VectorXd trq_ub_;

   public:
    AtlasWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~AtlasWorldNode();

    void customPreStep() override;
};
