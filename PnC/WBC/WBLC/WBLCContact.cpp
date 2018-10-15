#include "PnC/WBC/WBLC/WBLCContact.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "Utils/Utilities.hpp"

WBLCContact::WBLCContact(RobotSystem* robot_,
                         const std::string & linkName_,
                         double mu_): mIsUpdated(false),
                                      mFzUb(1000.),
                                      mFzLb(0.) {
    mRobot = robot_;
    mLinkName = linkName_;
    mMu = mu_;
    mJc = Eigen::MatrixXd::Zero(6, mRobot->getNumDofs());
    mJcDotQDot = Eigen::VectorXd::Zero(6);
    if (mLinkName == "FixedBody") {
        mInEqVec = Eigen::VectorXd::Zero(1);
    } else {
        mInEqVec = Eigen::VectorXd::Zero(18);
    }
}

WBLCContact::~WBLCContact() {}

void WBLCContact::updateWBLCContactSpec() {

    // update Jc and JcDotQDot
    if (mLinkName == "FixedBody") {
        mJc.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
        mJcDotQDot.setZero();
        mInEqVec.setZero();
    } else {
        // update Contact Geometry
        mContactBodyNodeIso = mRobot->getBodyNodeIsometry(mLinkName);
        mRobot->getContactAspects(mLinkName, mContactShape, mContactCenterIso);
        mOffset = mContactCenterIso.translation()
            - mContactBodyNodeIso.translation();
        mOffset[2] += -mContactShape[2]/2;

        mJc = mRobot->getBodyNodeJacobian(mLinkName, mOffset);
        mJcDotQDot = mRobot->getBodyNodeJacobianDot(mLinkName, mOffset)
            * mRobot->getQdot();
        mInEqVec.setZero();
        mInEqVec[0] = -mFzLb;
        mInEqVec[1] = -mFzUb;
    }
    mIsUpdated = true;
}

Eigen::MatrixXd WBLCContact::getWrenchFace() {
    // Construct contact wrench
    if (mLinkName == "FixedBody") {
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(1, 6);
        return ret;
    } else {
        Eigen::Isometry3d conIso1(mContactCenterIso);
        Eigen::Isometry3d conIso2(mContactCenterIso);
        Eigen::Isometry3d conIso3(mContactCenterIso);
        Eigen::Isometry3d conIso4(mContactCenterIso);
        Eigen::Isometry3d contactSurfaceIso(mContactCenterIso);
        std::vector<PointContact*> pointContactList;
        pointContactList.resize(4);
        PointContact pointContact1( conIso1.translate(
                    Eigen::Vector3d( mContactShape[0]/2,
                        mContactShape[1]/2,
                        -mContactShape[2]/2)), mMu );
        pointContactList[0] = &pointContact1;
        PointContact pointContact2( conIso2.translate(
                    Eigen::Vector3d( mContactShape[0]/2,
                        -mContactShape[1]/2,
                        -mContactShape[2]/2)), mMu );
        pointContactList[1] = &pointContact2;
        PointContact pointContact3( conIso3.translate(
                    Eigen::Vector3d(-mContactShape[0]/2,
                        -mContactShape[1]/2,
                        -mContactShape[2]/2)), mMu );
        pointContactList[2] = &pointContact3;
        PointContact pointContact4( conIso4.translate(
                    Eigen::Vector3d(-mContactShape[0]/2,
                        mContactShape[1]/2,
                        -mContactShape[2]/2)), mMu );
        pointContactList[3] = &pointContact4;
        ContactWrench contactWrench(  contactSurfaceIso.translate(
                    Eigen::Vector3d(0., 0., -mContactShape[2]/2)),
                pointContactList);

        Eigen::MatrixXd Uf = contactWrench.getWrenchFace(contactSurfaceIso);
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(Uf.rows()+2, Uf.cols());
        ret.block(2, 0, Uf.rows(), Uf.cols()) = Uf;
        ret(0, 5) = 1.0; ret(1, 5) = -1.0;
        return ret;
    }
}

Eigen::VectorXd WBLCContact::getInEqVector() {
    return mInEqVec;
}
