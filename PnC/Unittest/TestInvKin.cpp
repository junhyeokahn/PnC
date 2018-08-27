#include "gtest/gtest.h"

#include <drake/common/find_resource.h>
#include <drake/multibody/ik_options.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_constraint.h>
#include <drake/multibody/rigid_body_ik.h>
#include <drake/multibody/rigid_body_tree.h>

#include "eigen_matrix_compare.h"
#include "Configuration.h"
#include "RobotSystem.hpp"

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;


/* Finds and returns the indices within the state vector of @p tree that contain
 * the position states of a joint named @p name. The model instance ID is
 * ignored in this search (joints belonging to all model instances are
 * searched).
 */
std::vector<int> GetJointPositionVectorIndices(
    const RigidBodyTree<double>& tree, const std::string& name) {
  RigidBody<double>* joint_child_body = tree.FindChildBodyOfJoint(name);
  int num_positions = joint_child_body->getJoint().get_num_positions();
  std::vector<int> ret(static_cast<size_t>(num_positions));

  // Since the joint position states are located in a contiguous region of the
  // the rigid body tree's state vector, fill the return vector with
  // sequentially increasing indices starting at
  // `joint_child_body->get_position_start_index()`.
  iota(ret.begin(), ret.end(), joint_child_body->get_position_start_index());
  return ret;
}

void findJointAndInsert(const RigidBodyTree<double>& model,
                        const std::string& name,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                        std::vector<int>& position_list) {
  auto position_indices = GetJointPositionVectorIndices(model, name);

  position_list.insert(position_list.end(), position_indices.begin(),
                       position_indices.end());
}

TEST(InvKin, test1){

    //////////////////////
    // load draco to drake
    //////////////////////
    auto model = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDrake.urdf",
                drake::multibody::joints::kFixed, model.get());
    const Eigen::Vector2d tspan(0, 1);
    Eigen::VectorXd q0 = model->getZeroConfiguration();
    q0(2) = 0.8;
    const Eigen::Vector3d com_lb(0, 0, 1.0);
    const Eigen::Vector3d com_ub(0, 0, 1.0);
    const WorldCoMConstraint com_kc(model.get(), com_lb, com_ub, tspan);
    const std::vector<const RigidBodyConstraint*> constraint_array{&com_kc};
    const IKoptions ikoptions(model.get());
    Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(model->get_num_positions());
    int info = 0;
    std::vector<std::string> infeasible_constraint;
    inverseKin(model.get(), q0, q0, constraint_array.size(),
            constraint_array.data(), ikoptions, &q_sol, &info,
            &infeasible_constraint);
    EXPECT_EQ(info, 1);

    const KinematicsCache<double> cache = model->doKinematics(q_sol);
    const Eigen::Vector3d com = model->centerOfMass(cache);
    EXPECT_TRUE(
            CompareMatrices(com, Eigen::Vector3d(0, 0, 1), 1e-6,
                drake::MatrixCompareType::absolute));
}

TEST(InvKin, test2) {
    //////////////////////
    // load draco to drake
    //////////////////////
    auto model = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDrake.urdf",
                drake::multibody::joints::kFixed, model.get());

    //////////////
    // constraints
    //////////////

    // 1. Foot position constraint
    const Eigen::Vector2d tspan(0, 1);
    Eigen::VectorXd q0 = model->getZeroConfiguration();
    const double posTol(0.001);
    const Eigen::Vector3d rFootPos(0., -0.1, 0.065);
    const Eigen::Vector3d rFootPosLb = rFootPos - Eigen::Vector3d::Constant(posTol);
    const Eigen::Vector3d rFootPosUb = rFootPos + Eigen::Vector3d::Constant(posTol);
    const Eigen::Vector3d lFootPos(0., 0.1, 0.065);
    const Eigen::Vector3d lFootPosLb = lFootPos - Eigen::Vector3d::Constant(posTol);
    const Eigen::Vector3d lFootPosUb = lFootPos + Eigen::Vector3d::Constant(posTol);
    const int rFootIdx = model->FindBodyIndex("rAnkle");
    const int lFootIdx = model->FindBodyIndex("lAnkle");
    WorldPositionConstraint rFwpc(model.get(), rFootIdx,
            Vector3d::Zero(), rFootPosLb, rFootPosUb, tspan);
    WorldPositionConstraint lFwpc(model.get(), lFootIdx,
            Vector3d::Zero(), lFootPosLb, lFootPosUb, tspan);

    // 2. Foot rotation constraint
    Eigen::Vector4d quat_des(1, 0, 0, 0);
    double quatTol = 0.0017453292519943296;
    WorldQuatConstraint rFwqc(model.get(), rFootIdx, quat_des, quatTol,
            tspan);
    WorldQuatConstraint lFwqc(model.get(), lFootIdx, quat_des, quatTol,
            tspan);

    // 3. Joint limit constraint
    const double torsoRotTol(0.00001);
    PostureConstraint torsoRotConst(model.get(), tspan);
    std::vector<int> torsoRotIdx;
    findJointAndInsert(*model, "baseRotX", torsoRotIdx);
    findJointAndInsert(*model, "baseRotY", torsoRotIdx);
    findJointAndInsert(*model, "baseRotZ", torsoRotIdx);
    Eigen::Vector3d torsoRotLb
        = Eigen::Vector3d::Zero() - Eigen::Vector3d::Constant(torsoRotTol);
    Eigen::Vector3d torsoRotUb
        = Eigen::Vector3d::Zero() + Eigen::Vector3d::Constant(torsoRotTol);
    torsoRotConst.setJointLimits(3, torsoRotIdx.data(), torsoRotLb,
            torsoRotUb);

    // 4. quasi static constraints
    QuasiStaticConstraint quasiConst(model.get(), tspan);
    quasiConst.setShrinkFactor(0.2);
    quasiConst.setActive(true);
    Eigen::Matrix3Xd lFootPts = Eigen::Matrix3Xd::Zero(3, 4);
    lFootPts << 0.075, -0.075, 0.075, -0.075,
                 0.12,   0.12,  0.08,   0.08,
                0.065,  0.065, 0.065,  0.065;
    Eigen::Matrix3Xd rFootPts = Eigen::Matrix3Xd::Zero(3, 4);
    rFootPts << 0.075, -0.075, 0.075, -0.075,
                -0.12,  -0.12, -0.08,  -0.08,
                0.065,  0.065, 0.065,  0.065;
    quasiConst.addContact(1, &rFootIdx, &rFootPts);
    quasiConst.addContact(1, &lFootIdx, &lFootPts);

    ///////////
    // solve ik
    ///////////
    const std::vector<const RigidBodyConstraint*> constraint_array{&rFwpc,
        //&lFwpc, &rFwqc, &lFwqc, &torsoRotConst, &quasiConst};
        &lFwpc, &rFwqc, &lFwqc};
    const IKoptions ikoptions(model.get());
    VectorXd q_sol = VectorXd::Zero(model->get_num_positions());
    int info = 0;
    std::vector<std::string> infeasible_constraint;
    inverseKin(model.get(), q0, q0, constraint_array.size(),
            constraint_array.data(), ikoptions, &q_sol, &info,
            &infeasible_constraint);
    //approximateIK(model.get(), q0, q0, constraint_array.size(),
            //constraint_array.data(), ikoptions, &q_sol, &info);

    ///////////
    // validate
    ///////////
    ASSERT_EQ(info, 1);
    const KinematicsCache<double> cache = model->doKinematics(q_sol);
    EXPECT_TRUE(CompareMatrices(
                rFootPos, model->relativeTransform(cache, 0, rFootIdx).translation(),
                posTol + 2e-6, drake::MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(
                lFootPos, model->relativeTransform(cache, 0, lFootIdx).translation(),
                posTol + 2e-6, drake::MatrixCompareType::absolute));
    Eigen::Quaternion<double> lFQuat(model->relativeTransform(cache, 0, lFootIdx).linear());
    Eigen::Quaternion<double> rFQuat(model->relativeTransform(cache, 0, rFootIdx).linear());
    Eigen::Vector4d lFQuat_(lFQuat.w(), lFQuat.x(), lFQuat.y(), lFQuat.z());
    Eigen::Vector4d rFQuat_(rFQuat.w(), rFQuat.x(), rFQuat.y(), rFQuat.z());
    EXPECT_TRUE(CompareMatrices(
                quat_des, lFQuat_, quatTol + 2e-6, drake::MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(
                quat_des, rFQuat_, quatTol + 2e-6, drake::MatrixCompareType::absolute));
}
