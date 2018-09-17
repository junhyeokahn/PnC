#include "RobotSystem/RobotSystem.hpp"
#include <chrono>
#include "Utils/Utilities.hpp"

RobotSystem::RobotSystem(int numVirtual_, std::string file) {
    dart::utils::DartLoader urdfLoader;
    mSkel = urdfLoader.parseSkeleton(file);
    mNumDof = mSkel->getNumDofs();
    mNumVirtualDof = numVirtual_;
    mNumActuatedDof = mNumDof - mNumVirtualDof;
    mIcent = Eigen::MatrixXd::Zero(6, 6);
    mJcent = Eigen::MatrixXd::Zero(6, mNumDof);
    mAcent = Eigen::MatrixXd::Zero(6, mNumDof);
    printf("[Robot Model] Constructed\n");
}

RobotSystem::~RobotSystem() {}

Eigen::MatrixXd RobotSystem::getMassMatrix() {
    return mSkel->getMassMatrix();
}

Eigen::MatrixXd RobotSystem::getInvMassMatrix() {
    return mSkel->getInvMassMatrix();
}
Eigen::VectorXd RobotSystem::getCoriolisGravity() {
    return mSkel->getCoriolisAndGravityForces();
}

Eigen::VectorXd RobotSystem::getCoriolis() {
    return mSkel->getCoriolisForces();
}

Eigen::VectorXd RobotSystem::getGravity() {
    return mSkel->getGravityForces();
}

Eigen::MatrixXd RobotSystem::getCentroidJacobian() {
    return mJcent;
}

Eigen::MatrixXd RobotSystem::getCentroidInertia() {
    return mIcent;
}

Eigen::Vector3d RobotSystem::getCoMPosition() {
    return mSkel->getCOM();
}

Eigen::Vector3d RobotSystem::getCoMVelocity() {
    return mSkel->getCOMLinearVelocity();
}

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(const std::string & name_) {
    return mSkel->getBodyNode(name_)->getWorldTransform();
}

Eigen::Isometry3d RobotSystem::getBodyNodeCoMIsometry(const std::string & name_) {
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    ret.linear() = mSkel->getBodyNode(name_)->getWorldTransform().linear();
    ret.translation() = mSkel->getBodyNode(name_)->getCOM();
    return ret;
}

Eigen::Vector6d RobotSystem::getBodyNodeSpatialVelocity(const std::string & name_,
                                                        dart::dynamics::Frame* rl_,
                                                        dart::dynamics::Frame* wrt_) {
    return mSkel->getBodyNode(name_)->getSpatialVelocity(rl_, wrt_);
}

Eigen::Vector6d RobotSystem::getBodyNodeCoMSpatialVelocity(const std::string & name_,
                                                           dart::dynamics::Frame* rl_,
                                                           dart::dynamics::Frame* wrt_) {
    return mSkel->getBodyNode(name_)->getCOMSpatialVelocity(rl_, wrt_);
}

Eigen::VectorXd RobotSystem::getCentroidVelocity() {
    return mJcent * mSkel->getVelocities();
}

Eigen::MatrixXd RobotSystem::getCoMJacobian(dart::dynamics::Frame* wrt_) {
    return mSkel->getCOMJacobian(wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const std::string & name_,
                                    Eigen::Vector3d localOffset_,
                                    dart::dynamics::Frame* wrt_) {
    return mSkel->getJacobian(mSkel->getBodyNode(name_),
                              localOffset_,
                              wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobianDot(const std::string & name_,
                                       Eigen::Vector3d localOffset_,
                                       dart::dynamics::Frame* wrt_) {
    //return mSkel->getJacobianSpatialDeriv(mSkel->getBodyNode(name_),
                                          //localOffset_,
                                          //wrt_);

    return mSkel->getJacobianClassicDeriv(mSkel->getBodyNode(name_),
                                          localOffset_,
                                          wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobian(const std::string & name_,
                                       dart::dynamics::Frame* wrt_) {
    return mSkel->getJacobian(mSkel->getBodyNode(name_),
                              mSkel->getBodyNode(name_)->getLocalCOM(),
                              wrt_);
}

Eigen::MatrixXd RobotSystem::getBodyNodeCoMJacobianDot(const std::string & name_,
                                          dart::dynamics::Frame* wrt_) {
    //return mSkel->getJacobianSpatialDeriv(mSkel->getBodyNode(name_),
                                          //mSkel->getBodyNode(name_)->getLocalCOM(),
                                          //wrt_);

    return mSkel->getJacobianClassicDeriv(mSkel->getBodyNode(name_),
                                          mSkel->getBodyNode(name_)->getLocalCOM(),
                                          wrt_);
}

int RobotSystem::getJointIdx(const std::string & jointName_) {
    return mSkel->getJoint(jointName_)->getJointIndexInSkeleton();
}

int RobotSystem::getDofIdx(const std::string & dofName_) {
    return mSkel->getDof(dofName_)->getIndexInSkeleton();
}

void RobotSystem::getContactAspects(const std::string & linkName_,
                                    Eigen::Vector3d & size_,
                                    Eigen::Isometry3d & iso_) {

    dart::dynamics::ShapeNode* sn = mSkel->getBodyNode(linkName_)->
        getShapeNodesWith<dart::dynamics::CollisionAspect>().front();
    auto shape = sn->getShape();
    auto box = std::static_pointer_cast<dart::dynamics::BoxShape>(shape);
    size_ = box->getSize();
    Eigen::Isometry3d Tgb = getBodyNodeIsometry(linkName_);
    Eigen::Isometry3d Tbc;
    Tbc.translation() = sn->getRelativeTranslation();
    Tbc.linear() = sn->getRelativeRotation();
    iso_ = Tgb * Tbc;
}

void RobotSystem::updateSystem(double time_,
                               const Eigen::VectorXd & q_,
                               const Eigen::VectorXd & qdot_,
                               bool isUpdatingCentroid) {
    mTime = time_;
    mSkel->setPositions(q_);
    mSkel->setVelocities(qdot_);
    if (isUpdatingCentroid)  _updateCentroidFrame(q_, qdot_);
    mSkel->computeForwardKinematics();
}

//void RobotSystem::updateSystem(double time_,
                               //const Eigen::VectorXd & q_,
                               //const Eigen::VectorXd & qdot_,
                               //const Eigen::VectorXd & qdot_prev_,
                               //bool isUpdatingCentroid) {
    //mTime = time_;
    //mSkel->setPositions(q_);
    //if (myUtils::isEqual(qdot_, qdot_prev_)) {
        
    //}
    //mSkel->setVelocities(qdot_);
    //if (isUpdatingCentroid)  _updateCentroidFrame(q_, qdot_);
    //mSkel->computeForwardKinematics();
//}


void RobotSystem::_updateCentroidFrame(const Eigen::VectorXd & q_,
                                       const Eigen::VectorXd & qdot_) {
    Eigen::MatrixXd Jsp = Eigen::MatrixXd::Zero(6, mNumDof);
    Eigen::VectorXd p_gl = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd R_gl = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd pCoM_g = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd I = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Isometry3d T_lc = Eigen::Isometry3d::Identity();
    Eigen::MatrixXd AdT_lc = Eigen::MatrixXd::Zero(6, 6);
    mIcent = Eigen::MatrixXd::Zero(6, 6);
    mJcent = Eigen::MatrixXd::Zero(6, mNumDof);
    mAcent = Eigen::MatrixXd::Zero(6, mNumDof);
    pCoM_g = mSkel->getCOM();
    for (int i = 0; i < mSkel->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = mSkel->getBodyNode(i);
        Jsp = mSkel->getJacobian(bn);
        p_gl = bn->getWorldTransform().translation();
        R_gl = bn->getWorldTransform().linear();
        I = bn->getSpatialInertia();
        T_lc.linear() = R_gl.transpose();
        T_lc.translation() = R_gl.transpose() * (pCoM_g - p_gl);
        AdT_lc = dart::math::getAdTMatrix(T_lc);
        mIcent += AdT_lc.transpose() * I * AdT_lc;
        mAcent += AdT_lc.transpose() * I * Jsp;
    }
    mJcent = mIcent.inverse() * mAcent;
}

//For debugging puropse
Eigen::VectorXd RobotSystem::rotateVector( const Eigen::VectorXd & vec ) {

    Eigen::VectorXd ret1 = vec;
    Eigen::Isometry3d iso_gl = mSkel->getRootBodyNode()->getWorldTransform();
    Eigen::MatrixXd augR = Eigen::MatrixXd::Zero(6, 6);
    augR.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    augR.block(3, 3, 3, 3) = iso_gl.linear();
    ret1.head(6) = augR * vec.head(6);
    Eigen::VectorXd ret = ret1;
    (ret.head(6)).tail(3) = ret1.head(3);
    ret.head(3) = (ret1.head(6)).tail(3);
    return ret;
}

Eigen::MatrixXd RobotSystem::rotateJacobian( const Eigen::MatrixXd & jac) {

    // rotate root
    Eigen::Isometry3d iso_gl = mSkel->getRootBodyNode()->getWorldTransform();
    Eigen::MatrixXd ret1 = jac;
    Eigen::MatrixXd augR = Eigen::MatrixXd::Zero(6, 6);
    augR.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    augR.block(3, 3, 3, 3) = (iso_gl.linear()).transpose();
    ret1.block(0, 0, 6, 6) = ret1.block(0, 0, 6, 6) * augR;

    // switch rot & lin
    Eigen::MatrixXd ret = ret1;
    ret.block(0, 0, 6, 3) = ret1.block(0, 3, 6, 3);
    ret.block(0, 3, 6, 3) = ret1.block(0, 0, 6, 3);
    return ret;
}
