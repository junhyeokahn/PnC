#include "WBLCContact.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"

WBLCContact::WBLCContact(RobotSystem* robot_,
                         const std::string & linkName_,
                         double mu_): mIsUpdated(false) {
    mRobot = robot_;
    mLinkName = linkName_;
    mContactBodyNodeIso = mRobot->getBodyNodeIsometry(mLinkName);
    mRobot->getContactAspects(linkName_, mContactShape, mContactCenterIso);
    mOffset = mContactCenterIso.translation()
        - mContactBodyNodeIso.translation();
    mOffset[2] += -mContactShape[2]/2;
    // Construct Contact Wrench
    Eigen::Isometry3d conIso1(mContactCenterIso);
    Eigen::Isometry3d conIso2(mContactCenterIso);
    Eigen::Isometry3d conIso3(mContactCenterIso);
    Eigen::Isometry3d conIso4(mContactCenterIso);
    mContactSurfaceIso = mContactCenterIso;
    mPointContactList.resize(4);
    mPointContactList[0] = new PointContact(conIso1.translate(
                Eigen::Vector3d( mContactShape[0]/2,
                                 mContactShape[1]/2,
                                -mContactShape[2]/2)), mu_);
    mPointContactList[1] = new PointContact(conIso2.translate(
                Eigen::Vector3d( mContactShape[0]/2,
                                -mContactShape[1]/2,
                                -mContactShape[2]/2)), mu_);
    mPointContactList[2] = new PointContact(conIso3.translate(
                Eigen::Vector3d(-mContactShape[0]/2,
                                -mContactShape[1]/2,
                                -mContactShape[2]/2)), mu_);
    mPointContactList[3] = new PointContact(conIso4.translate(
                Eigen::Vector3d(-mContactShape[0]/2,
                                 mContactShape[1]/2,
                                -mContactShape[2]/2)), mu_);
    mContactWrench = new ContactWrench(mContactSurfaceIso.translate(
                Eigen::Vector3d(0., 0., -mContactShape[2]/2)),
                mPointContactList);

    mJc = Eigen::MatrixXd::Zero(6, mRobot->getNumDofs());
    mJcDotQDot = Eigen::VectorXd::Zero(6);
}

WBLCContact::~WBLCContact() {
    for (int i = 0; i < 4; ++i)
        delete mPointContactList[i];
    delete mContactWrench;
}

void WBLCContact::updateWBLCContactSpec() {
    // update Jc and JcDotQDot
    mJc = mRobot->getBodyNodeJacobian(mLinkName, mOffset);
    mJcDotQDot = mRobot->getBodyNodeJacobianDot(mLinkName, mOffset)
        * mRobot->getQdot();
    mIsUpdated = true;
}

Eigen::MatrixXd WBLCContact::getWrenchFace() {
    Eigen::MatrixXd Uf = mContactWrench->getWrenchFace(mContactSurfaceIso);
    Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(Uf.rows()+1, Uf.cols());
    ret.block(1, 0, Uf.rows(), Uf.cols()) = Uf;
    ret(0, 5) = 1.0;
    return ret;
}
