#include "gtest/gtest.h"

#include <drake/common/find_resource.h>
#include <drake/multibody/ik_options.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_constraint.h>
#include <drake/multibody/rigid_body_ik.h>
#include <drake/multibody/rigid_body_tree.h>

#include "Utils/eigen_matrix_compare.h"
#include "Configuration.h"
#include "RobotSystem/RobotSystem.hpp"

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

TEST(TestDrakeDart, floatingTest) {
    auto drake_model = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoFixedNoVisualDrake.urdf",
                drake::multibody::joints::kRollPitchYaw, drake_model.get());
    auto cache = std::make_unique<KinematicsCache<double>>(
            drake_model->CreateKinematicsCache());
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr dart_model = urdfLoader.parseSkeleton(
            THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDart.urdf");
    //for (int i = 0; i < dart_model->getNumBodyNodes(); ++i) {
        //auto& rigid_body = drake_model->get_body(i+2);
        //dart::dynamics::BodyNodePtr bn = dart_model->getBodyNode(i);
        //EXPECT_EQ(rigid_body.get_name(), bn->getName());
        //EXPECT_EQ(rigid_body.get_mass(), bn->getMass());
        //std::cout << rigid_body.get_name() << std::endl;
    //}
    for (int i = 0; i < drake_model->get_num_bodies(); ++i) {
        auto& rigid_body = drake_model->get_body(i);
        std::cout << rigid_body.get_name() << std::endl;
    }
}

TEST(TestDrakeDart, LoadURDF) {

    //////////////////////
    // load draco to drake
    //////////////////////
    auto drake_model = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDrake.urdf",
                drake::multibody::joints::kFixed, drake_model.get());
    auto cache = std::make_unique<KinematicsCache<double>>(
            drake_model->CreateKinematicsCache());


    /////////////////////
    // load draco to dart
    /////////////////////
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr dart_model = urdfLoader.parseSkeleton(
            THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDart.urdf");

    //////////////////
    // compare numbers
    //////////////////
    EXPECT_EQ(drake_model->get_num_bodies()-2, dart_model->getNumBodyNodes());
    for (int i = 0; i < dart_model->getNumBodyNodes(); ++i) {
        auto& rigid_body = drake_model->get_body(i+2);
        dart::dynamics::BodyNodePtr bn = dart_model->getBodyNode(i);
        EXPECT_EQ(rigid_body.get_name(), bn->getName());
        EXPECT_EQ(rigid_body.get_mass(), bn->getMass());
    }
    EXPECT_EQ(drake_model->get_num_positions(), dart_model->getNumDofs());
    EXPECT_EQ(drake_model->get_num_velocities(), dart_model->getNumDofs());
    EXPECT_EQ(drake_model->get_num_actuators(), dart_model->getNumDofs() - 6);

    //////////////////
    // compare fwd kin
    //////////////////
    Eigen::VectorXd q; q.resize(drake_model->get_num_positions());
    Eigen::VectorXd qdot; qdot.resize(drake_model->get_num_velocities());
    q.setZero(); qdot.setZero();
    q[7] = 1.2; qdot[11] = 1.;
    //double tol = 10 * Eigen::NumTraits<double>::epsilon();
    double tol = 0.00000001;
    cache->initialize(q, qdot); drake_model->doKinematics(*cache);
    dart_model->setPositions(q); dart_model->setVelocities(qdot); dart_model->computeForwardKinematics();
    for (int i = 0; i < dart_model->getNumBodyNodes(); ++i) {
        auto& rigid_body = drake_model->get_body(i+2);
        dart::dynamics::BodyNodePtr bn = dart_model->getBodyNode(i);
        Eigen::Isometry3d drake_iso = drake_model->CalcBodyPoseInWorldFrame(
                *cache, rigid_body);
        Eigen::Isometry3d dart_iso = bn->getWorldTransform();
        EXPECT_TRUE(drake::CompareMatrices(drake_iso.linear(), dart_iso.linear(),
                    tol, drake::MatrixCompareType::absolute));
        EXPECT_TRUE(drake::CompareMatrices(drake_iso.translation(), dart_iso.translation(),
                    tol, drake::MatrixCompareType::absolute));
        EXPECT_TRUE(drake::CompareMatrices(drake_model->CalcBodySpatialVelocityInWorldFrame(*cache, rigid_body),
                    bn->getSpatialVelocity(dart::dynamics::Frame::World(), dart::dynamics::Frame::World()),
                    tol, drake::MatrixCompareType::absolute));
        EXPECT_TRUE(drake::CompareMatrices(drake_model->CalcBodySpatialVelocityJacobianInWorldFrame(*cache, rigid_body),
                    dart_model->getJacobian(bn, Eigen::Vector3d::Zero(3), dart::dynamics::Frame::World()),
                    tol, drake::MatrixCompareType::absolute));

    }
    EXPECT_TRUE(drake::CompareMatrices(drake_model->centerOfMass(*cache), dart_model->getCOM(),
            tol, drake::MatrixCompareType::absolute));
    EXPECT_TRUE(drake::CompareMatrices(drake_model->centerOfMassJacobian(*cache),
                dart_model->getCOMLinearJacobian(dart::dynamics::Frame::World()),
                tol, drake::MatrixCompareType::absolute));

    ///////////////////
    // compare dynamics
    ///////////////////
    const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
    EXPECT_TRUE(drake::CompareMatrices(drake_model->dynamicsBiasTerm(*cache, no_external_wrenches, false),
                dart_model->getGravityForces(),
                tol, drake::MatrixCompareType::absolute));
}

TEST(TestDrakeDart, CentroidDynamics){
    //////////////////////
    // load draco to drake
    //////////////////////
    auto drake_model = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDrake.urdf",
                drake::multibody::joints::kFixed, drake_model.get());
    auto cache = std::make_unique<KinematicsCache<double>>(
            drake_model->CreateKinematicsCache());


    ///////////////////// load draco to dart ///////////////////
    auto dart_model = std::make_unique<RobotSystem>(6,
            THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDart.urdf");

    Eigen::VectorXd q; q.resize(drake_model->get_num_positions());
    Eigen::VectorXd qdot; qdot.resize(drake_model->get_num_velocities());
    q.setZero(); qdot.setZero();
    q[7] = 1.2; qdot[11] = 1.;
    double tol = 0.00000001;
    cache->initialize(q, qdot); drake_model->doKinematics(*cache);
    dart_model->updateSystem(0.0, q, qdot, true);
    EXPECT_TRUE(drake::CompareMatrices(drake_model->centroidalMomentumMatrix(*cache),
                dart_model->getCentroidInertia()*dart_model->getCentroidJacobian(),
                tol, drake::MatrixCompareType::absolute));
}
