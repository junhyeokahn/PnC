#include "gtest/gtest.h"

#include <drake/common/find_resource.h>
#include <drake/multibody/ik_options.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_constraint.h>
#include <drake/multibody/rigid_body_ik.h>
#include <drake/multibody/rigid_body_tree.h>
#include "drake/math/eigen_sparse_triplet.h"
#include <Utilities.hpp>

#include "eigen_matrix_compare.h"
#include "Configuration.h"
#include "RobotSystem.hpp"

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "BSplineBasic.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;


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
        &lFwpc, &rFwqc};
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
    //EXPECT_TRUE(CompareMatrices(
                //quat_des, lFQuat_, quatTol + 2e-6, drake::MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(
                quat_des, rFQuat_, quatTol + 2e-6, drake::MatrixCompareType::absolute));
}

TEST(InvKin, test3) {

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

    // time span and seeds
    Vector2d tspan(0, 1);
    int nT = 4;
    double dt = tspan(1) / (nT - 1);
    std::vector<double> t(nT, 0);
    for (int i = 0; i < nT; i++) {
        t[i] = dt * i;
    }
    std::vector<Vector2d> tspanList = { Vector2d(t[0], t[0]),
                                        Vector2d(t[1], t[1]),
                                        Vector2d(t[2], t[2]),
                                        Vector2d(t[3], t[3]) };
    Eigen::VectorXd qstar = model->getZeroConfiguration();
    MatrixXd q0 = qstar.replicate(1, nT);
    VectorXd qdot0 = VectorXd::Zero(model->get_num_velocities());
    std::vector<RigidBodyConstraint*> constraint_array;

    // 1. Foot position constraint
    double posTol(0.001);
    Eigen::Vector3d rFootPos(0., -0.1, 0.065);
    Eigen::Vector3d rFootPosLb = rFootPos - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d rFootPosUb = rFootPos + Eigen::Vector3d::Constant(posTol);
    std::vector<Eigen::Vector3d> lFootPosList = { Eigen::Vector3d(0., 0.05, 0.1),
                                                  Eigen::Vector3d(0., 0.05, 0.1),
                                                  Eigen::Vector3d(0., 0.05, 0.1),
                                                  Eigen::Vector3d(0., 0.05, 0.1)};
    std::vector<Eigen::Vector3d> lFootPosLbList(4);
    std::vector<Eigen::Vector3d> lFootPosUbList(4);
    for (int i = 0; i < 4; ++i) {
        lFootPosLbList[i] = lFootPosList[i] - Eigen::Vector3d::Constant(posTol);
        lFootPosUbList[i] = lFootPosList[i] + Eigen::Vector3d::Constant(posTol);
    }
    int rFootIdx = model->FindBodyIndex("rAnkle");
    int lFootIdx = model->FindBodyIndex("lAnkle");
    WorldPositionConstraint rFwpc(model.get(), rFootIdx,
            Vector3d::Zero(), rFootPosLb, rFootPosUb, tspan);
    constraint_array.push_back(&rFwpc);
    std::vector<WorldPositionConstraint> lFwpcList;
    for (int i = 0; i < nT; ++i) {
        lFwpcList.push_back(WorldPositionConstraint(model.get(), lFootIdx, Vector3d::Zero(),
                lFootPosLbList[i], lFootPosUbList[i], tspanList[i]));
        //constraint_array.push_back(&(lFwpcList[i]));
    }

    // 2. CoM constraint
    Eigen::Vector3d com_lb(0, -0.1, std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d com_ub(0, -0.1, std::numeric_limits<double>::quiet_NaN());
    WorldCoMConstraint com_kc(model.get(), com_lb, com_ub, tspan);
    constraint_array.push_back(&com_kc);

    ///////////
    // solve ik
    ///////////
    IKoptions ikoptions(model.get());
    ikoptions.setFixInitialState(false);
    ikoptions.setMajorIterationsLimit(500);

    MatrixXd q_sol(model->get_num_positions(), nT);
    MatrixXd qdot_sol(model->get_num_velocities(), nT);
    MatrixXd qddot_sol(model->get_num_positions(), nT);
    int info = 0;
    std::vector<std::string> infeasible_constraint;

    inverseKinTraj(model.get(), nT, t.data(), qdot0, q0, q0,
            constraint_array.size(), constraint_array.data(), ikoptions,
            &q_sol, &qdot_sol, &qddot_sol, &info, &infeasible_constraint);
    EXPECT_EQ(info, 1);

    /////////////
    // validation
    /////////////
    //std::cout << q_sol << std::endl;
    //std::cout << "--" << std::endl;
    for (int i = 0; i < nT; ++i) {
        Eigen::VectorXd q = q_sol.block(i, 0, q_sol.rows(), 1);
        Eigen::VectorXd qdot = qdot_sol = qdot_sol.block(i, 0, qdot_sol.rows(), 1);
        Eigen::VectorXd qddot = qdot_sol = qddot_sol.block(i, 0, qddot_sol.rows(), 1);
        KinematicsCache<double> cache = model->doKinematics(q);
        Eigen::Isometry3d rFootIsoRes = model->CalcBodyPoseInWorldFrame(
                cache, *model->get_bodies()[rFootIdx] );
        Eigen::Isometry3d lFootIsoRes = model->CalcBodyPoseInWorldFrame(
                cache, *model->get_bodies()[lFootIdx] );
        Eigen::Vector3d comRes = model->centerOfMass(cache);
        //std::cout << "-------------------------------------------" << std::endl;
        //std::cout << i << " th Result" << std::endl;
        //std::cout << "rFoot" << std::endl;
        //std::cout << rFootIsoRes.translation() << std::endl;
        //std::cout << "lFoot" << std::endl;
        //std::cout << lFootIsoRes.translation() << std::endl;
        //std::cout << "com" << std::endl;
        //std::cout << comRes << std::endl;
    }
}

TEST(InvKin, test4) {
  auto model = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      drake::FindResourceOrThrow(
          "drake/examples/atlas/urdf/atlas_minimal_contact.urdf"),
      drake::multibody::joints::kRollPitchYaw, model.get());

  int r_hand{};
  int pelvis{};
  for (int i = 0; i < static_cast<int>(model->get_bodies().size()); i++) {
    if (!model->get_bodies()[i]->get_name().compare(std::string("r_hand"))) {
      r_hand = i;
    }
    if (!model->get_bodies()[i]->get_name().compare(std::string("pelvis"))) {
      pelvis = i;
    }
  }

  VectorXd qstar = model->getZeroConfiguration();
  qstar(3) = 0.8;
  KinematicsCache<double> cache = model->doKinematics(qstar);
  Vector3d com0 = model->centerOfMass(cache);

  Vector3d r_hand_pt = Vector3d::Zero();
  Vector3d rhand_pos0 = model->transformPoints(cache, r_hand_pt, r_hand, 0);

  //std::cout << "1" << std::endl;
  //std::cout << rhand_pos0 << std::endl;
  //std::cout << "2" << std::endl;
  //std::cout << model->get_bodies()[r_hand]->get_name() << std::endl;
  //std::cout << model->CalcBodyPoseInWorldFrame(cache, *model->get_bodies()[r_hand]).translation() << std::endl;

  Vector2d tspan(0, 1);
  int nT = 4;
  double dt = tspan(1) / (nT - 1);
  std::vector<double> t(nT, 0);
  for (int i = 0; i < nT; i++) {
    t[i] = dt * i;
  }
  MatrixXd q0 = qstar.replicate(1, nT);
  VectorXd qdot0 = VectorXd::Zero(model->get_num_velocities());
  Vector3d com_lb = com0;
  com_lb(0) = std::numeric_limits<double>::quiet_NaN();
  com_lb(1) = std::numeric_limits<double>::quiet_NaN();
  Vector3d com_ub(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      com0(2) + 0.5);
  WorldCoMConstraint com_kc(model.get(), com_lb, com_ub);
  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) += 0.1;
  rhand_pos_lb(1) += 0.05;
  rhand_pos_lb(2) += 0.25;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.25;
  Vector2d tspan_end;
  tspan_end << t[nT - 1], t[nT - 1];
  WorldPositionConstraint kc_rhand(
      model.get(), r_hand, r_hand_pt, rhand_pos_lb, rhand_pos_ub, tspan_end);

  // Add a multiple time constraint which is fairly trivial to meet in
  // this case.
  WorldFixedBodyPoseConstraint kc_fixed_pose(model.get(), pelvis, tspan);

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&com_kc);
  constraint_array.push_back(&kc_rhand);
  constraint_array.push_back(&kc_fixed_pose);

  IKoptions ikoptions(model.get());
  MatrixXd q_sol(model->get_num_positions(), nT);
  MatrixXd qdot_sol(model->get_num_velocities(), nT);
  MatrixXd qddot_sol(model->get_num_positions(), nT);
  int info = 0;
  std::vector<std::string> infeasible_constraint;

  inverseKinTraj(model.get(), nT, t.data(), qdot0, q0, q0,
                 constraint_array.size(), constraint_array.data(), ikoptions,
                 &q_sol, &qdot_sol, &qddot_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);

  ikoptions.setFixInitialState(false);
  ikoptions.setMajorIterationsLimit(500);

  inverseKinTraj(model.get(), nT, t.data(), qdot0, q0, q0,
                 constraint_array.size(), constraint_array.data(), ikoptions,
                 &q_sol, &qdot_sol, &qddot_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);

  Eigen::RowVectorXd t_inbetween(5);
  t_inbetween << 0.1, 0.15, 0.3, 0.4, 0.6;
  ikoptions.setAdditionaltSamples(t_inbetween);
  inverseKinTraj(model.get(), nT, t.data(), qdot0, q0, q0,
                 constraint_array.size(), constraint_array.data(), ikoptions,
                 &q_sol, &qdot_sol, &qddot_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);
}

TEST(InvKin, test5) {

    //////////////////////
    // load draco to drake
    //////////////////////
    auto model = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDrake.urdf",
                drake::multibody::joints::kFixed, model.get());

    //////////////////////////
    // generate CoM trajectory
    //////////////////////////
    BS_Basic<3, 3, 0, 2, 2> spline;
    double ini[9] = {0.0, 0.0, 0.8,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0};
    double fin[9] = {0.0, 0.0, 0.7,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0};
    double **middle_pt;
    spline.SetParam(ini, fin, middle_pt, 1.0);

    int num_knot(100);

    ////////////////////////////////
    // desired task space trajectory
    ////////////////////////////////

    std::vector<double> t(num_knot);
    std::vector< Eigen::Vector3d > com_pos(num_knot);
    std::vector< Eigen::Vector3d > com_vel(num_knot);
    Eigen::VectorXd t_eigen =  Eigen::VectorXd::LinSpaced(num_knot, 0., 1.);
    double d_ary[3];
    for (int i = 0; i < num_knot; ++i) {
        t[i] = t_eigen[i];
        spline.getCurvePoint(t[i], d_ary);
        for (int j = 0; j < 3; ++j) com_pos[i][j] = d_ary[j];
        spline.getCurveDerPoint(t[i], 1, d_ary);
        for (int j = 0; j < 3; ++j) com_vel[i][j] = d_ary[j];
    }
    int rf_idx = model->FindBodyIndex("rAnkle");
    int lf_idx = model->FindBodyIndex("lAnkle");
    double posTol = 0.0001;
    Eigen::Vector3d des_rf_pos; des_rf_pos << -0.021, -0.167, 0.065;
    Eigen::Vector3d des_rf_pos_lb; des_rf_pos_lb = des_rf_pos - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d des_rf_pos_ub; des_rf_pos_ub = des_rf_pos + Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d des_lf_pos; des_lf_pos << -0.021, 0.167, 0.065;
    Eigen::Vector3d des_lf_pos_lb; des_lf_pos_lb = des_lf_pos - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d des_lf_pos_ub; des_lf_pos_ub = des_lf_pos + Eigen::Vector3d::Constant(posTol);
    double quatTol = 0.0017453292519943296;
    Eigen::Vector4d des_quat; des_quat << 1, 0, 0, 0;

    Eigen::VectorXd qInit = Eigen::VectorXd::Zero(16);
    qInit <<
        0.000971709,
        0.000161351,
        1.3235,
        -0.000809759,
        0.00724312,
        -0.000535272,
        0.000347594,
        0.000567963,
        -0.425732,
        0.832267,
        1.15919,
        0.000876023,
        0.000505368,
        -0.425368,
        0.831448,
        1.16017;
    std::vector< Eigen::VectorXd > q_sol(num_knot, Eigen::VectorXd::Zero(16));

    Eigen::Vector2d tspan(0, 1);
    for (int i = 0; i < num_knot; ++i) {
        std::cout << "**** " << i << " th Time Step Solver ****" << std::endl;
        std::vector<RigidBodyConstraint*> constraint_array;
        // 1. foot position & orientation constraint
        WorldPositionConstraint rfp(model.get(), rf_idx, Eigen::Vector3d::Zero(), des_rf_pos - Eigen::Vector3d::Constant(posTol), des_rf_pos + Eigen::Vector3d::Constant(posTol), tspan);
        constraint_array.push_back(&rfp);
        WorldPositionConstraint lfp(model.get(), lf_idx, Eigen::Vector3d::Zero(), des_lf_pos - Eigen::Vector3d::Constant(posTol), des_lf_pos + Eigen::Vector3d::Constant(posTol), tspan);
        constraint_array.push_back(&lfp);
        WorldQuatConstraint rfq(model.get(), rf_idx, des_quat, quatTol, tspan);
        constraint_array.push_back(&rfq);
        WorldQuatConstraint lfq(model.get(), lf_idx, des_quat, quatTol, tspan);
        constraint_array.push_back(&lfq);
        // 2. com position
        WorldCoMConstraint comp(model.get(), com_pos[i] - Eigen::Vector3d::Constant(posTol), com_pos[i] + Eigen::Vector3d::Constant(posTol), tspan);
        constraint_array.push_back(&comp);
        // 3. foot velocities
        Eigen::MatrixXd j_com(3, 16);
        Eigen::MatrixXd j_rf_lf(12, 16);
        Eigen::VectorXd q_vec(16);
        if (i == 0) {
            KinematicsCache<double> cache = model->doKinematics(qInit);
            q_vec = qInit;
            j_rf_lf.block(0, 0, 6, 16) = model->CalcBodySpatialVelocityJacobianInWorldFrame(cache, model->get_body(rf_idx));
            j_rf_lf.block(6, 0, 6, 16) = model->CalcBodySpatialVelocityJacobianInWorldFrame(cache, model->get_body(lf_idx));
            j_com = model->centerOfMassJacobian(cache);
        } else {
            KinematicsCache<double> cache = model->doKinematics(q_sol[i-1]);
            q_vec = q_sol[i-1];
            j_rf_lf.block(0, 0, 6, 16) = model->CalcBodySpatialVelocityJacobianInWorldFrame(cache, model->get_body(rf_idx));
            j_rf_lf.block(6, 0, 6, 16) = model->CalcBodySpatialVelocityJacobianInWorldFrame(cache, model->get_body(lf_idx));
            j_com = model->centerOfMassJacobian(cache);
        }

        Eigen::VectorXi rows;
        Eigen::VectorXi cols;
        Eigen::VectorXd vals;
        double dt(t[1] - t[0]);
        double velTol(0.01);
        Eigen::VectorXd q_q_vec(32);
        q_q_vec << q_vec, q_vec;
        myUtils::collectNonZeroIdxAndValue(j_rf_lf, rows, cols, vals);
        SingleTimeLinearPostureConstraint fvc(model.get(), rows, cols, vals,
                j_rf_lf*q_q_vec - Eigen::VectorXd::Constant(12, velTol)*dt,
                j_rf_lf*q_q_vec + Eigen::VectorXd::Constant(12, velTol)*dt,
                tspan);
        //constraint_array.push_back(&fvc);
        // 4. com velocities
        myUtils::collectNonZeroIdxAndValue(j_com, rows, cols, vals);
        SingleTimeLinearPostureConstraint cvc(model.get(), rows, cols, vals,
                com_vel[i]*dt + j_com*q_vec - Eigen::VectorXd::Constant(3, velTol)*dt,
                com_vel[i]*dt + j_com*q_vec + Eigen::VectorXd::Constant(3, velTol)*dt,
                tspan);
        constraint_array.push_back(&cvc);

        IKoptions ikoptions(model.get());
        ikoptions.setMajorIterationsLimit(1500);
        ikoptions.setIterationsLimit(20000);
        std::vector<std::string> infeasible_constraint;
        int info = 0;
        if (i == 0) {
         inverseKin(model.get(), qInit, qInit, constraint_array.size(),
                constraint_array.data(), ikoptions, &(q_sol[i]), &info,
                &infeasible_constraint);
        } else {
         inverseKin(model.get(), q_sol[i-1], q_sol[i-1], constraint_array.size(),
                constraint_array.data(), ikoptions, &(q_sol[i]), &info,
                &infeasible_constraint);
        }
        EXPECT_EQ(info, 1);
    }

    // check solution
    double additional_tol(1e-5);
    for (int i = 0; i < num_knot; ++i) {
        std::cout << "**** " << i << " th Time Step Check ****" << std::endl;
        KinematicsCache<double> cache = model->doKinematics(q_sol[i]);
        Eigen::Isometry3d rf_iso = model->relativeTransform(cache, 0, rf_idx);
        Eigen::Quaternion<double> rf_quat_(rf_iso.linear());
        Eigen::Vector4d rf_quat(rf_quat_.w(), rf_quat_.x(), rf_quat_.y(), rf_quat_.z());
        Eigen::Isometry3d lf_iso = model->relativeTransform(cache, 0, lf_idx);
        Eigen::Quaternion<double> lf_quat_(lf_iso.linear());
        Eigen::Vector4d lf_quat(lf_quat_.w(), lf_quat_.x(), lf_quat_.y(), lf_quat_.z());
        Eigen::Vector3d com = model->centerOfMass(cache);
        EXPECT_TRUE(CompareMatrices(com, com_pos[i], posTol + additional_tol, drake::MatrixCompareType::absolute));
        EXPECT_TRUE(CompareMatrices(des_rf_pos, rf_iso.translation(), posTol + additional_tol, drake::MatrixCompareType::absolute));
        EXPECT_TRUE(CompareMatrices(des_lf_pos, lf_iso.translation(), posTol + additional_tol, drake::MatrixCompareType::absolute));
        EXPECT_TRUE(CompareMatrices(des_quat, lf_quat, quatTol + additional_tol, drake::MatrixCompareType::absolute));
        EXPECT_TRUE(CompareMatrices(des_quat, rf_quat, quatTol + additional_tol, drake::MatrixCompareType::absolute));
    }

}
