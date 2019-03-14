#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>

AtlasStateProvider* AtlasStateProvider::getStateProvider(RobotSystem* _robot) {
    static AtlasStateProvider state_provider_(_robot);
    return &state_provider_;
}

AtlasStateProvider::AtlasStateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "Atlas State Provider");

    robot_ = _robot;
    stance_foot = AtlasBodyNode::l_sole;
    curr_time = 0.;

    q = Eigen::VectorXd::Zero(Atlas::n_dof);
    qdot = Eigen::VectorXd::Zero(Atlas::n_dof);
    global_pos_local.setZero();

    b_rfoot_contact = 0;
    b_lfoot_contact = 0;

    des_location.setZero();
    des_quat = Eigen::Quaternion<double>::Identity();
    walking_velocity.setZero();

    rfoot_pos.setZero();
    lfoot_pos.setZero();
    rfoot_quat = Eigen::Quaternion<double>::Identity();
    lfoot_quat = Eigen::Quaternion<double>::Identity();
    rfoot_vel.setZero();
    lfoot_vel.setZero();
    rfoot_so3.setZero();
    lfoot_so3.setZero();

    DataManager* data_manager = DataManager::GetDataManager();

    data_manager->RegisterData(&curr_time, DOUBLE, "time");
    data_manager->RegisterData(&q, VECT, "config", Atlas::n_dof);
    data_manager->RegisterData(&qdot, VECT, "qdot", Atlas::n_dof);
    data_manager->RegisterData(&global_pos_local, VECT3, "global_pos_local", 3);

    data_manager->RegisterData(&b_rfoot_contact, INT, "rfoot_contact", 1);
    data_manager->RegisterData(&b_lfoot_contact, INT, "lfoot_contact", 1);

    data_manager->RegisterData(&rfoot_pos, VECT3, "rfoot_pos", 3);
    data_manager->RegisterData(&lfoot_pos, VECT3, "lfoot_pos", 3);
    data_manager->RegisterData(&rfoot_vel, VECT3, "rfoot_vel", 3);
    data_manager->RegisterData(&lfoot_vel, VECT3, "lfoot_vel", 3);
    data_manager->RegisterData(&rfoot_so3, VECT3, "rfoot_so3", 3);
    data_manager->RegisterData(&lfoot_so3, VECT3, "lfoot_so3", 3);
    data_manager->RegisterData(&rfoot_quat, QUATERNION, "rfoot_quat", 4);
    data_manager->RegisterData(&lfoot_quat, QUATERNION, "lfoot_quat", 4);
    data_manager->RegisterData(&des_quat, QUATERNION, "des_quat", 4);
}

void AtlasStateProvider::saveCurrentData() {
    rfoot_pos =
        robot_->getBodyNodeIsometry(AtlasBodyNode::r_sole).translation();
    lfoot_pos =
        robot_->getBodyNodeIsometry(AtlasBodyNode::l_sole).translation();
    rfoot_vel =
        robot_->getBodyNodeSpatialVelocity(AtlasBodyNode::r_sole).tail(3);
    lfoot_vel =
        robot_->getBodyNodeSpatialVelocity(AtlasBodyNode::l_sole).tail(3);

    rfoot_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(AtlasBodyNode::r_sole).linear());
    lfoot_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(AtlasBodyNode::l_sole).linear());
    rfoot_so3 =
        robot_->getBodyNodeSpatialVelocity(AtlasBodyNode::r_sole).head(3);
    lfoot_so3 =
        robot_->getBodyNodeSpatialVelocity(AtlasBodyNode::l_sole).head(3);
}
