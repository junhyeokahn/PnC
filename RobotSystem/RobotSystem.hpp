#pragma once

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <Eigen/Dense>
#include <stdio.h>

class RobotSystem
{
protected:
    dart::dynamics::SkeletonPtr skel_ptr_;
    int num_dof_;
    int num_virtual_dof_;
    int num_actuated_dof_;
    Eigen::MatrixXd I_cent_;
    Eigen::MatrixXd J_cent_;
    Eigen::MatrixXd A_cent_;

    /*
     * Update I_cent_, A_cent_, J_cent_
     * , where
     * centroid_momentum = I_cent_ * centroid_velocity = A_cent_ * qdot
     *           J_cent_ = inv(I_cent_) * A_cent_
     * centroid_velocity = J_cent_ * qdot
     */
    void _updateCentroidFrame(const Eigen::VectorXd & q_,
                              const Eigen::VectorXd & qdot_);

public:
    RobotSystem(int numVirtual_, std::string file);
    virtual ~RobotSystem(void);

    dart::dynamics::SkeletonPtr getSkeleton() { return skel_ptr_; };
    dart::dynamics::BodyNodePtr getBodyNode(const std::string & _link_name) { return skel_ptr_->getBodyNode(_link_name); }
    Eigen::VectorXd getQ() { return skel_ptr_->getPositions(); };
    Eigen::VectorXd getQdot() { return skel_ptr_->getVelocities(); };
    double getRobotMass() { return skel_ptr_->getMass(); }
    int getNumDofs() { return num_dof_; };
    int getNumVirtualDofs() { return num_virtual_dof_; };
    int getNumActuatedDofs() { return num_actuated_dof_; };

    int getJointIdx(const std::string & jointName_);
    int getDofIdx(const std::string & dofName_);

    Eigen::MatrixXd getMassMatrix();
    Eigen::MatrixXd getInvMassMatrix();
    Eigen::VectorXd getGravity();
    Eigen::VectorXd getCoriolis();
    Eigen::VectorXd getCoriolisGravity();

    Eigen::MatrixXd getCentroidJacobian();
    Eigen::MatrixXd getCentroidInertiaTimesJacobian();
    Eigen::MatrixXd getCentroidInertia();
    Eigen::VectorXd getCentroidVelocity();
    Eigen::VectorXd getCentroidMomentum();
    Eigen::Vector3d getCoMPosition(dart::dynamics::Frame* wrt_
                                   =dart::dynamics::Frame::World());
    Eigen::Vector3d getCoMVelocity(dart::dynamics::Frame* rl_
                                   =dart::dynamics::Frame::World(),
                                   dart::dynamics::Frame* wrt_
                                   =dart::dynamics::Frame::World());
    Eigen::MatrixXd getCoMJacobian(dart::dynamics::Frame* wrt_
                                   =dart::dynamics::Frame::World());
    void updateSystem(const Eigen::VectorXd & q_,
                      const Eigen::VectorXd & qdot_,
                      bool isUpdatingCentroid_ = true);

    Eigen::Isometry3d getBodyNodeIsometry(const std::string & name_,
                                          dart::dynamics::Frame* wrt_
                                          =dart::dynamics::Frame::World());
    Eigen::Isometry3d getBodyNodeCoMIsometry(const std::string & name_,
                                             dart::dynamics::Frame* wrt_
                                             =dart::dynamics::Frame::World());
    Eigen::Isometry3d getBodyNodeCollisionIsometry(const std::string & name_,
                                                   dart::dynamics::Frame* wrt_
                                                   =dart::dynamics::Frame::World());
    Eigen::Vector3d getBodyNodeCollisionShape(const std::string & _link_name);
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

};
