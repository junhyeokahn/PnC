#ifndef ROBOTSYSTEM_H
#define ROBOTSYSTEM_H

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <Eigen/Dense>
#include <stdio.h>

class RobotSystem
{
protected:
    dart::dynamics::SkeletonPtr mSkel;
    int mNumDof;
    int mNumVirtualDof;
    int mNumActuatedDof;
    Eigen::VectorXd mInitialConfiguration;
    double mTime;
    Eigen::MatrixXd mIcent;
    Eigen::MatrixXd mJcent;
    Eigen::MatrixXd mAcent;

    /*
     * Update Ig, Ag, Jg
     * , where h = [k, l]' = I_{cm} * \dot{x}_{cm}
     *                     = A_{cm} * \dot{q}
     *              J_{cm} = I_{cm}^{-1} * A_{cm}
     *        \dot{x}_{cm} = J_{cm} * \dot{q}
     */
    void _updateCentroidFrame(const Eigen::VectorXd & q_,
                              const Eigen::VectorXd & qdot_);

public:
    RobotSystem(int numVirtual_, std::string file);
    virtual ~RobotSystem(void);

    Eigen::MatrixXd getMassMatrix();
    Eigen::MatrixXd getInvMassMatrix();
    Eigen::VectorXd getGravity();
    Eigen::VectorXd getCoriolis();
    Eigen::VectorXd getCoriolisGravity();
    Eigen::MatrixXd getCentroidJacobian();
    Eigen::MatrixXd getCentroidInertia();
    Eigen::Vector3d getCoMPosition();
    Eigen::Vector3d getCoMVelocity();
    Eigen::Isometry3d getBodyNodeIsometry(const std::string & name_);
    Eigen::Isometry3d getBodyNodeCoMIsometry(const std::string & name_);
    Eigen::Vector6d getBodyNodeSpatialVelocity(const std::string & name_,
                                               dart::dynamics::Frame* rl_
                                               =dart::dynamics::Frame::World(),
                                               dart::dynamics::Frame* wrt_
                                               =dart::dynamics::Frame::World());
    Eigen::Vector6d getBodyNodeCoMSpatialVelocity(const std::string & name_,
                                                  dart::dynamics::Frame* rl_
                                                  =dart::dynamics::Frame::World(),
                                                  dart::dynamics::Frame* wrt_
                                                  =dart::dynamics::Frame::World());
    int getJointIdx(const std::string & jointName_);
    int getDofIdx(const std::string & dofName_);

    /*
     * linkName_ : name of Link
     * size_ : size of Contact
     * iso_ : SE3 of Center of Contact from Global
     */
    void getContactAspects(const std::string & linkName_,
                           Eigen::Vector3d & size_,
                           Eigen::Isometry3d & iso_);
    Eigen::VectorXd getCentroidVelocity();
    Eigen::MatrixXd getCoMJacobian(dart::dynamics::Frame* wrt_
                                   =dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeJacobian(const std::string & name_,
                                        Eigen::Vector3d localOffset_
                                        =Eigen::Vector3d::Zero(3),
                                        dart::dynamics::Frame * wrt_
                                        =dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeJacobianDot(const std::string & name_,
                                           Eigen::Vector3d localOffset_
                                           =Eigen::Vector3d::Zero(3),
                                           dart::dynamics::Frame * wrt_
                                           =dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeCoMJacobian(const std::string & name_,
                                           dart::dynamics::Frame * wrt_
                                           =dart::dynamics::Frame::World());
    Eigen::MatrixXd getBodyNodeCoMJacobianDot(const std::string & name_,
                                              dart::dynamics::Frame * wrt_
                                              =dart::dynamics::Frame::World());
    void updateSystem(double time_,
                      const Eigen::VectorXd & q_,
                      const Eigen::VectorXd & qdot_,
                      bool isUpdatingCentroid_ = true);

    dart::dynamics::SkeletonPtr getSkeleton() { return mSkel; };
    Eigen::VectorXd getQ() { return mSkel->getPositions(); };
    Eigen::VectorXd getQdot() { return mSkel->getVelocities(); };
    void setInitialConfiguration(const Eigen::VectorXd q) { mInitialConfiguration = q; };
    Eigen::VectorXd getInitialConfiguration() { return mInitialConfiguration; }
    double getTime() { return mTime; }
    int getNumDofs() { return mNumDof; };
    int getNumVirtualDofs() { return mNumVirtualDof; };
    int getNumActuatedDofs() { return mNumActuatedDof; };

    Eigen::VectorXd rotateVector( const Eigen::VectorXd & vec );
    Eigen::MatrixXd rotateJacobian( const Eigen::MatrixXd & mat );
};

#endif /* ROBOTSYSTEM_H */
