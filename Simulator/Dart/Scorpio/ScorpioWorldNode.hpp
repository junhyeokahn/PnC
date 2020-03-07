#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <unsupported/Eigen/CXX11/Tensor>

class EnvInterface;
class ScorpioSensorData;
class ScorpioCommand;
class Scorpio2SensorData;
class Scorpio2Command;
class DracoSensorData;
class DracoCommand;

/*! \enum 
 *
 *  Detailed description
 */
enum BoxPH {
    table=0,
    scorpio=1,
    draco=2,
    scorpio2=3,
    table2=4
};

class ScorpioWorldNode : public dart::gui::osg::WorldNode {
   private:

    EnvInterface* scorpio_interface_;
    ScorpioSensorData* scorpio_sensordata_;
    ScorpioCommand* scorpio_cmd_;

    EnvInterface* scorpio_interface2_;
    Scorpio2SensorData* scorpio_sensordata2_;
    Scorpio2Command* scorpio_cmd2_;

    void SetParams_();
    void GetForceTorqueData_();

    EnvInterface* draco_interface_;
    DracoSensorData* draco_sensordata_;
    DracoCommand* draco_cmd_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr scorpio_;
    dart::dynamics::SkeletonPtr scorpio2_;
    dart::dynamics::SkeletonPtr draco_;
    //dart::dynamics::SkeletonPtr mSkel_hsr_;
    dart::dynamics::SkeletonPtr mGround_;
    dart::dynamics::SkeletonPtr mbox_;

    dart::dynamics::JointPtr active1_;
    dart::dynamics::JointPtr active2_;
    dart::dynamics::JointPtr active3_;
    dart::dynamics::JointPtr active4_;
    dart::dynamics::JointPtr active5_;
    dart::dynamics::JointPtr active6_;
    dart::dynamics::JointPtr active7_;

    dart::dynamics::JointPtr active1__;
    dart::dynamics::JointPtr active2__;
    dart::dynamics::JointPtr active3__;
    dart::dynamics::JointPtr active4__;
    dart::dynamics::JointPtr active5__;
    dart::dynamics::JointPtr active6__;
    dart::dynamics::JointPtr active7__;

    Eigen::VectorXd active_joint_idx_;
    Eigen::VectorXd passive_joint_idx_;
    Eigen::VectorXd q_init_;

    Eigen::VectorXd Amp_;
    Eigen::VectorXd Freq_;

    Eigen::VectorXd scorpio_kp_;
    Eigen::VectorXd scorpio_kd_;
    Eigen::VectorXd scorpio_trq_cmd_;
    Eigen::VectorXd scorpio_trq_cmd2_;

    Eigen::VectorXd draco_kp_;
    Eigen::VectorXd draco_kd_;
    Eigen::VectorXd draco_trq_cmd_;
    bool b_plot_mpc_result_;

    int draco_n_dof_;

    //Eigen::VectorXd cmd_hsr_;

    int count_;
    int sim_case_;
    int control_type_;
    double servo_rate_;
    int n_dof_scorpio_;
    int a_dof_scorpio_;
    int p_dof_scorpio_;
    //int n_dof_hsr_;
    int actuator_type_;
    //int actuator_type_hsr_;
    double t_;

    Eigen::VectorXd a_joint_idx_scorpio_;
    Eigen::VectorXd p_joint_idx_scorpio_;

    bool draco_first_is_done;
    bool draco_second_is_done;
    bool first_scorpio_done;
    bool second_scorpio_done;
    double current_box_height_;
    double draco_top_height_;

    void SetActiveForce(const Eigen::VectorXd & des_force);
    void SetActiveForce2(const Eigen::VectorXd & des_force);

    void print_position();

    void GetContactSwitchData_(bool&, bool&);
    void PlotMPCResult_();

   public:
    ScorpioWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~ScorpioWorldNode();

    void customPreStep() override;

    Eigen::VectorXd p1_;
    Eigen::VectorXd p2_;
    double x_;
    double y_;
    double th_;
    double x2_;
    Eigen::VectorXd box_kp;
    Eigen::VectorXd box_kd;

    Eigen::VectorXd des_box_pos;
    Eigen::VectorXd des_box_pos_6d;

    Eigen::VectorXd box_ini_pos;
    Eigen::VectorXd box_fin_pos;

    Eigen::VectorXd first_scorpio_ini_pos;
    Eigen::VectorXd second_scorpio_ini_pos;

    BoxPH box_ph;
};
