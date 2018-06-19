#ifndef CONTACT_H
#define CONTACT_H

#include <Eigen/Dense>
#include <vector>
#include <dart/dart.hpp>

class Polytope;

class PointContact
{
public:

    /*
     * isometry : SE(3) from global frame to local frame
     * mu : friction coefficient
     */
    PointContact(const Eigen::Isometry3d & isometry_, double mu_);
    virtual ~PointContact();

    /*
     * H-representation of force friction cone in local frame
     * Matrix (4, 3)
     * Usage : H-rep * force_local < 0
     */
    Eigen::MatrixXd getForceFace();

    /*
     * Matrix (3, 4) : [f1_local, f2_local, f3_local, f4_local]
     * Usage : [f1_local, f2_local, f3_local, f4_local] * [a; b; c; d]
     * Return force in local frame
     */
    Eigen::MatrixXd getForceSpan();

    /*
     * Get SE(3) from global frame to local frame
     */
    Eigen::Isometry3d getLocalFrame();

private:
    Eigen::Isometry3d mLocalFrame;
    double mMu;
};

class ContactWrench
{
public:
    /*
     * appliedFrame : SE(3) from global frame to local frame
     *                for Contact Wrench construction
     * pointContactList : PointContact List
     */
    ContactWrench (const Eigen::Isometry3d & appliedFrame_,
                   const std::vector<PointContact*> & pointContactLists_);
    virtual ~ContactWrench ();

    /*
     * H-representation of contact wrench in wrt_ frame (default: local frame)
     * Computed thorugh double description from V-representation
     * F * wrench > 0
     */
    Eigen::MatrixXd getWrenchFace(Eigen::Isometry3d wrt_
                                  =Eigen::Isometry3d::Identity());

    /*
     * V-representation of contact wrench in local frame
     * Usage : [basis] * [scalar]'
     * Matrix (6, 4*numContact)
     */
    Eigen::MatrixXd getWrenchSpan();

    int getNumContactPoint();

protected:
    std::vector<PointContact*> mContactPointLists;
    Polytope* mPolytope;
    Eigen::Isometry3d mAppliedFrame;
    int mNumContact;
    Eigen::MatrixXd mV;
    Eigen::MatrixXd mU;

    void _computeV();
};

class ApproximatedCWC
{
public:
    ApproximatedCWC ( const Eigen::MatrixXd & P_,
                      const Eigen::VectorXd & d_,
                      const Eigen::MatrixXd & SO_,
                      double nullValue_);
    virtual ~ApproximatedCWC ();

    Eigen::MatrixXd getP();
    Eigen::VectorXd getb();
    Eigen::MatrixXd getSO();
    double getNullDimValue();

private:
    /* data */
    Eigen::MatrixXd mP;
    Eigen::VectorXd mb;
    Eigen::MatrixXd mSO;
    double mNullValue;
};

#endif /* CONTACT_H */
