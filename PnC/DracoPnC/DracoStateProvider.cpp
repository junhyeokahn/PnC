#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <RobotSystem/RobotSystem.hpp>
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
    q = Eigen::VectorXd::Zero(robot_->getNumDofs());
    qdot = Eigen::VectorXd::Zero(robot_->getNumDofs());
    rotor_inertia = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    global_pos_local.setZero();
    des_location.setZero();
    est_mocap_body_pos.setZero();
    est_mocap_body_vel.setZero();
    b_rfoot_contact = 0;
    b_lfoot_contact = 0;
    // qddot_cmd = Eigen::VectorXd::Zero(16);
    reaction_forces = Eigen::VectorXd::Zero(12);
    led_kin_data = Eigen::VectorXd::Zero(9);

    rfoot_contact_center_pos.setZero();
    lfoot_contact_center_pos.setZero();
    rfoot_contact_center_vel.setZero();
    lfoot_contact_center_vel.setZero();

    first_LED_x = 0.;
    first_LED_y = 0.;

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

    data_manager->RegisterData(&rfoot_contact_center_pos, VECT3, "rfoot_cp", 3);
    data_manager->RegisterData(&lfoot_contact_center_pos, VECT3, "lfoot_cp", 3);
    data_manager->RegisterData(&rfoot_contact_center_vel, VECT3, "rfoot_cv", 3);
    data_manager->RegisterData(&lfoot_contact_center_vel, VECT3, "lfoot_cv", 3);

    data_manager->RegisterData(&com_pos, VECT3, "com_pos", 3);
    data_manager->RegisterData(&com_vel, VECT3, "com_vel", 3);
    data_manager->RegisterData(&com_vel, VECT3, "est_com_vel", 3);

    data_manager->RegisterData(&est_mocap_body_pos, VECT3, "est_mocap_body_pos",
                               3);
    data_manager->RegisterData(&est_mocap_body_vel, VECT2, "est_mocap_body_vel",
                               2);
}

void DracoStateProvider::saveCurrentData() {
    rfoot_contact_center_pos =
        robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
    lfoot_contact_center_pos =
        robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
    rfoot_contact_center_vel =
        robot_->getBodyNodeSpatialVelocity(DracoBodyNode::rFootCenter).tail(3);
    lfoot_contact_center_vel =
        robot_->getBodyNodeSpatialVelocity(DracoBodyNode::lFootCenter).tail(3);
}
