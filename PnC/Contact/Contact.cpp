#include "Contact.hpp"
#include <Polytope.h>
#include <iostream>
#include <dart/dart.hpp>

PointContact::PointContact(const Eigen::Isometry3d & localFrame_, double mu_) {
    mLocalFrame = localFrame_;
    mMu = mu_;
}

PointContact::~PointContact() {}

Eigen::MatrixXd PointContact::getForceFace() {

    Eigen::MatrixXd face_local(4,3);
    face_local << -1,  0, -mMu,
                   1,  0, -mMu,
                   0, -1, -mMu,
                   0,  1, -mMu;

    return face_local;
}

Eigen::MatrixXd PointContact::getForceSpan(){
    Eigen::MatrixXd ret(3, 4);

    ret << mMu, mMu , -mMu, -mMu,
           mMu, -mMu, mMu , -mMu,
           1. , 1.  , 1.  , 1.  ;

    return ret;
}

Eigen::Isometry3d PointContact::getLocalFrame() {
    return mLocalFrame;
}

ContactWrench::ContactWrench(const Eigen::Isometry3d & appliedFrame_,
                             const std::vector<PointContact*> & pointContactLists_) {

    mContactPointLists = pointContactLists_;
    mPolytope = new Polytope();
    mAppliedFrame = appliedFrame_;
    mNumContact = pointContactLists_.size();
    mV.resize(6, 4*mNumContact);
}

ContactWrench::~ContactWrench() {}

Eigen::MatrixXd ContactWrench::getWrenchSpan() {
    _computeV();
    return mV;
}

Eigen::MatrixXd ContactWrench::getWrenchFace(Eigen::Isometry3d wrt_) {
    _computeV();
    mPolytope->hrep(mV.transpose(), Eigen::VectorXd::Zero(mV.cols()));
    mU = -mPolytope->hrep().first;
    Eigen::MatrixXd augR = Eigen::MatrixXd::Zero(6, 6);
    augR.block(0, 0, 3, 3) = wrt_.linear().transpose();
    augR.block(3, 3, 3, 3) = wrt_.linear().transpose();
    mU *= augR;
    return mU;
}

void ContactWrench::_computeV() {
    Eigen::MatrixXd augAdjoint = Eigen::MatrixXd::Zero(6, 3*mNumContact);
    Eigen::MatrixXd augVpoint = Eigen::MatrixXd::Zero(3*mNumContact, 4*mNumContact);

    for (int i = 0; i < mNumContact; ++i) {
        augAdjoint.block(0, 3*i, 6, 3) = (dart::math::getAdTMatrix(
                    (mContactPointLists[i]->getLocalFrame().inverse())*mAppliedFrame
                    ).transpose()).block(0, 3, 6, 3);
        augVpoint.block(3*i, 4*i, 3, 4) = mContactPointLists[i]->getForceSpan();
    }
    mV = augAdjoint * augVpoint;
}

int ContactWrench::getNumContactPoint() {
    return mNumContact;
}

ApproximatedCWC::ApproximatedCWC ( const Eigen::MatrixXd & P_,
                                   const Eigen::VectorXd & b_,
                                   const Eigen::MatrixXd & SO_,
                                   double nullValue_) {
    mP = P_;
    mb = b_;
    mSO = SO_;
    mNullValue = nullValue_;
}

ApproximatedCWC::~ApproximatedCWC() {}

Eigen::MatrixXd ApproximatedCWC::getP() {
    return mP;
}

Eigen::VectorXd ApproximatedCWC::getb() {
    return mb;
}

Eigen::MatrixXd ApproximatedCWC::getSO() {
    return mSO;
}

double ApproximatedCWC::getNullDimValue() {
    return mNullValue;
}
