#include <PnC/RobotSystem/RobotSystem.hpp>
#include <chrono>

RobotSystem::RobotSystem(int numVirtual_, std::string file) {
    myUtils::pretty_constructor(1, "Robot Model");

    dart::utils::DartLoader urdfLoader;
    skel_ptr_ = urdfLoader.parseSkeleton(file);
    num_dof_ = skel_ptr_->getNumDofs();
    num_virtual_dof_ = numVirtual_;
    num_actuated_dof_ = num_dof_ - num_virtual_dof_;
    I_cent_ = Eigen::MatrixXd::Zero(6, 6);
    J_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
    A_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
}

RobotSystem::~RobotSystem() {}

Eigen::MatrixXd RobotSystem::getMassMatrix() {
    return skel_ptr_->getMassMatrix();
}

Eigen::MatrixXd RobotSystem::getInvMassMatrix() {
    return skel_ptr_->getInvMassMatrix();
}
Eigen::VectorXd RobotSystem::getCoriolisGravity() {
    return skel_ptr_->getCoriolisAndGravityForces();
}

Eigen::VectorXd RobotSystem::getCoriolis() {
    return skel_ptr_->getCoriolisForces();
}

Eigen::VectorXd RobotSystem::getGravity() {
    return skel_ptr_->getGravityForces();
}

Eigen::MatrixXd RobotSystem::getCentroidJacobian() { return J_cent_; }

Eigen::MatrixXd RobotSystem::getCentroidInertiaTimesJacobian() {
    return A_cent_;
}

Eigen::MatrixXd RobotSystem::getCentroidInertia() { return I_cent_; }

Eigen::Vector3d RobotSystem::getCoMPosition(dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getCOM(wrt_);
}

Eigen::Vector3d RobotSystem::getCoMVelocity(dart::dynamics::Frame* rl_,
                                            dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getCOMLinearVelocity(rl_, wrt_);
}

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getTransform(wrt_);
}

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getTransform(wrt_);
}

Eigen::Isometry3d RobotSystem::getBodyNodeCoMIsometry(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    ret.linear() = getBodyNodeIsometry(name_, wrt_).linear();
    ret.translation() = skel_ptr_->getBodyNode(name_)->getCOM(wrt_);
    return ret;
}

Eigen::Isometry3d RobotSystem::getBodyNodeCoMIsometry(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    ret.linear() = getBodyNodeIsometry(_bn_idx, wrt_).linear();
    ret.translation() = skel_ptr_->getBodyNode(_bn_idx)->getCOM(wrt_);
    return ret;
}

Eigen::Vector6d RobotSystem::getBodyNodeSpatialVelocity(
    const std::string& name_, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getSpatialVelocity(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeSpatialVelocity(
    const int& _bn_idx, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getSpatialVelocity(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeCoMSpatialVelocity(
    const std::string& name_, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(name_)->getCOMSpatialVelocity(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeCoMSpatialVelocity(
    const int& _bn_idx, dart::dynamics::Frame* rl_,
    dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getBodyNode(_bn_idx)->getCOMSpatialVelocity(rl_, wrt_);
}

Eigen::VectorXd RobotSystem::getCentroidVelocity() {
    return J_cent_ * skel_ptr_->getVelocities();
}

Eigen::VectorXd RobotSystem::getCentroidMomentum() {
    return A_cent_ * skel_ptr_->getVelocities();
}

Eigen::MatrixXd RobotSystem::getCoMJacobian(dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getCOMJacobian(wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const std::string& name_,
                                                 Eigen::Vector3d localOffset_,
                                                 dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(skel_ptr_->getBodyNode(name_), localOffset_,
                                  wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const int& _bn_idx,
                                                 Eigen::Vector3d localOffset_,
                                                 dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(skel_ptr_->getBodyNode(_bn_idx), localOffset_,
                                  wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobianDot(
    const std::string& name_, Eigen::Vector3d localOffset_,
    dart::dynamics::Frame* wrt_) {
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(name_),
    // localOffset_,
    // wrt_);

    return skel_ptr_->getJacobianClassicDeriv(skel_ptr_->getBodyNode(name_),
                                              localOffset_, wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobianDot(
    const int& _bn_idx, Eigen::Vector3d localOffset_,
    dart::dynamics::Frame* wrt_) {
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(name_),
    // localOffset_,
    // wrt_);

    return skel_ptr_->getJacobianClassicDeriv(skel_ptr_->getBodyNode(_bn_idx),
                                              localOffset_, wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobian(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(skel_ptr_->getBodyNode(name_),
                                  skel_ptr_->getBodyNode(name_)->getLocalCOM(),
                                  wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobian(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {
    return skel_ptr_->getJacobian(
        skel_ptr_->getBodyNode(_bn_idx),
        skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM(), wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobianDot(
    const std::string& name_, dart::dynamics::Frame* wrt_) {
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(name_),
    // skel_ptr_->getBodyNode(name_)->getLocalCOM(),
    // wrt_);

    return skel_ptr_->getJacobianClassicDeriv(
        skel_ptr_->getBodyNode(name_),
        skel_ptr_->getBodyNode(name_)->getLocalCOM(), wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobianDot(
    const int& _bn_idx, dart::dynamics::Frame* wrt_) {
    // return skel_ptr_->getJacobianSpatialDeriv(skel_ptr_->getBodyNode(name_),
    // skel_ptr_->getBodyNode(name_)->getLocalCOM(),
    // wrt_);

    return skel_ptr_->getJacobianClassicDeriv(
        skel_ptr_->getBodyNode(_bn_idx),
        skel_ptr_->getBodyNode(_bn_idx)->getLocalCOM(), wrt_);
}

int RobotSystem::getJointIdx(const std::string& jointName_) {
    return skel_ptr_->getJoint(jointName_)->getJointIndexInSkeleton();
}

int RobotSystem::getDofIdx(const std::string& dofName_) {
    return skel_ptr_->getDof(dofName_)->getIndexInSkeleton();
}

void RobotSystem::updateSystem(const Eigen::VectorXd& q_,
                               const Eigen::VectorXd& qdot_,
                               bool isUpdatingCentroid) {
    skel_ptr_->setPositions(q_);
    skel_ptr_->setVelocities(qdot_);
    if (isUpdatingCentroid) _updateCentroidFrame(q_, qdot_);
    skel_ptr_->computeForwardKinematics();
}

void RobotSystem::_updateCentroidFrame(const Eigen::VectorXd& q_,
                                       const Eigen::VectorXd& qdot_) {
    Eigen::MatrixXd Jsp = Eigen::MatrixXd::Zero(6, num_dof_);
    Eigen::VectorXd p_gl = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd R_gl = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd pCoM_g = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd I = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Isometry3d T_lc = Eigen::Isometry3d::Identity();
    Eigen::MatrixXd AdT_lc = Eigen::MatrixXd::Zero(6, 6);
    I_cent_ = Eigen::MatrixXd::Zero(6, 6);
    J_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
    A_cent_ = Eigen::MatrixXd::Zero(6, num_dof_);
    pCoM_g = skel_ptr_->getCOM();
    for (int i = 0; i < skel_ptr_->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(i);
        Jsp = skel_ptr_->getJacobian(bn);
        p_gl = bn->getWorldTransform().translation();
        R_gl = bn->getWorldTransform().linear();
        I = bn->getSpatialInertia();
        T_lc.linear() = R_gl.transpose();
        T_lc.translation() = R_gl.transpose() * (pCoM_g - p_gl);
        AdT_lc = dart::math::getAdTMatrix(T_lc);
        I_cent_ += AdT_lc.transpose() * I * AdT_lc;
        A_cent_ += AdT_lc.transpose() * I * Jsp;
    }
    J_cent_ = I_cent_.inverse() * A_cent_;
}

void RobotSystem::printRobotInfo() {
    for (int i = 0; i < skel_ptr_->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = skel_ptr_->getBodyNode(i);
        std::cout << "BodyNode::" << bn->getName() << std::endl;
    }
    for (int i = 0; i < skel_ptr_->getNumDofs(); ++i) {
        dart::dynamics::DegreeOfFreedom* dof = skel_ptr_->getDof(i);
        std::cout << "DoF::" << dof->getName() << std::endl;
    }
    exit(0);
}
