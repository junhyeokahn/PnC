#pragma once

#include "PnC/Contact/Contact.hpp"

class RobotSystem;
class PointContact;
class ContactWrench;

class WBLCContact
{
private:
    RobotSystem* mRobot;
    std::string mLinkName;
    Eigen::Isometry3d mContactBodyNodeIso;
    Eigen::Isometry3d mContactCenterIso;
    Eigen::Vector3d mContactShape;
    Eigen::VectorXd mOffset; // Vector from bodynode to contact point in local coordinate

    bool mIsUpdated;
    Eigen::MatrixXd mJc;
    Eigen::VectorXd mJcDotQDot;
    double mMu;
    Eigen::VectorXd mIEqVec;

public:
    WBLCContact(RobotSystem* robot_, const std::string & linkName_, double mu_);
    virtual ~WBLCContact();

    void updateWBLCContactSpec();
    void unsetWBLCContactSpec() { mIsUpdated = false; };
    Eigen::MatrixXd getJc() { return mJc; }
    Eigen::MatrixXd getJcDotQDot() { return mJcDotQDot; }
    bool isContactSpecSet() { return mIsUpdated; }
    Eigen::MatrixXd getWrenchFace();
    Eigen::VectorXd getIeqVector();
    std::string getContactBodyNodeName() { return mLinkName; }
};
