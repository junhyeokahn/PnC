#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>

DracoStateProvider* DracoStateProvider::getStateProvider(RobotSystem* _robot) {
    static DracoStateProvider state_provider_(_robot);
    return &state_provider_;
}

DracoStateProvider::DracoStateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "State Provider");

    robot_ = _robot;
    stance_foot = "lFoot";
    curr_time = 0.;

    target_yaw = 0.;
    rl_count = 0;
    adjusted_foot.setZero();
    guided_foot.setZero();
    walking_velocity.setZero();
    des_quat = Eigen::Quaternion<double>(1, 0, 0, 0);

    q = Eigen::VectorXd::Zero(robot_->getNumDofs());
    qdot = Eigen::VectorXd::Zero(robot_->getNumDofs());
    rotor_inertia = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    global_pos_local.setZero();
    des_location.setZero();
    est_mocap_body_pos.setZero();
    est_mocap_body_vel.setZero();
    b_rfoot_contact = 0;
    b_lfoot_contact = 0;
    qddot_cmd = Eigen::VectorXd::Zero(16);
    reaction_forces = Eigen::VectorXd::Zero(12);
    filtered_rf = Eigen::VectorXd::Zero(12);
    led_kin_data = Eigen::VectorXd::Zero(9);

    rfoot_center_pos.setZero();
    lfoot_center_pos.setZero();
    rfoot_center_vel.setZero();
    lfoot_center_vel.setZero();
    rfoot_center_quat = Eigen::Quaternion<double>::Identity();
    lfoot_center_quat = Eigen::Quaternion<double>::Identity();
    rfoot_center_so3.setZero();
    lfoot_center_so3.setZero();

    first_LED_x = 0.;
    first_LED_y = 0.;

    com_pos.setZero();
    com_vel.setZero();
    com_pos_des.setZero();
    com_vel_des.setZero();
    est_com_vel.setZero();

    mpc_pred_pos.setZero();
    mpc_pred_vel.setZero();

    dcm.setZero();
    omega = 0.7;

    DataManager* data_manager = DataManager::GetDataManager();

    data_manager->RegisterData(&curr_time, DOUBLE, "time");
    data_manager->RegisterData(&q, VECT, "config", robot_->getNumDofs());
    data_manager->RegisterData(&qdot, VECT, "qdot", robot_->getNumDofs());
    data_manager->RegisterData(&rotor_inertia, VECT, "rotor_inertia",
                               robot_->getNumActuatedDofs());
    data_manager->RegisterData(&global_pos_local, VECT3, "global_pos_local", 3);

    data_manager->RegisterData(&b_rfoot_contact, INT, "rfoot_contact", 1);
    data_manager->RegisterData(&b_lfoot_contact, INT, "lfoot_contact", 1);

    data_manager->RegisterData(&reaction_forces, VECT, "reaction_force", 12);
    data_manager->RegisterData(&filtered_rf, VECT, "filtered_rf", 12);
    data_manager->RegisterData(&qddot_cmd, VECT, "qddot_cmd", 16);

    data_manager->RegisterData(&rfoot_center_pos, VECT3, "rfoot_pos", 3);
    data_manager->RegisterData(&lfoot_center_pos, VECT3, "lfoot_pos", 3);
    data_manager->RegisterData(&rfoot_center_vel, VECT3, "rfoot_vel", 3);
    data_manager->RegisterData(&lfoot_center_vel, VECT3, "lfoot_vel", 3);
    data_manager->RegisterData(&rfoot_center_so3, VECT3, "rfoot_so3", 3);
    data_manager->RegisterData(&lfoot_center_so3, VECT3, "lfoot_so3", 3);
    data_manager->RegisterData(&rfoot_center_quat, QUATERNION, "rfoot_quat", 4);
    data_manager->RegisterData(&lfoot_center_quat, QUATERNION, "lfoot_quat", 4);
    data_manager->RegisterData(&des_quat, QUATERNION, "des_quat", 4);

    data_manager->RegisterData(&com_pos, VECT3, "com_pos", 3);
    data_manager->RegisterData(&com_vel, VECT3, "com_vel", 3);
    data_manager->RegisterData(&est_com_vel, VECT3, "est_com_vel", 3);

    data_manager->RegisterData(&mpc_pred_pos, VECT3, "mpc_pred_pos", 3);
    data_manager->RegisterData(&mpc_pred_vel, VECT3, "mpc_pred_vel", 3);

    data_manager->RegisterData(&com_pos_des, VECT3, "com_pos_des", 3);
    data_manager->RegisterData(&com_vel_des, VECT3, "com_vel_des", 3);

    data_manager->RegisterData(&est_mocap_body_pos, VECT3, "est_mocap_body_pos",
                               3);
    data_manager->RegisterData(&est_mocap_body_vel, VECT2, "est_mocap_body_vel",
                               2);

    data_manager->RegisterData(&dcm, VECT3, "dcm", 3);
}

void DracoStateProvider::saveCurrentData() {
    rfoot_center_pos =
        robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
    lfoot_center_pos =
        robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
    rfoot_center_vel =
        robot_->getBodyNodeSpatialVelocity(DracoBodyNode::rFootCenter).tail(3);
    lfoot_center_vel =
        robot_->getBodyNodeSpatialVelocity(DracoBodyNode::lFootCenter).tail(3);

    rfoot_center_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).linear());
    lfoot_center_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).linear());
    rfoot_center_so3 =
        robot_->getBodyNodeSpatialVelocity(DracoBodyNode::rFootCenter).head(3);
    lfoot_center_so3 =
        robot_->getBodyNodeSpatialVelocity(DracoBodyNode::lFootCenter).head(3);
}
