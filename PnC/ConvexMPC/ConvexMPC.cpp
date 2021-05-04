// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <PnC/ConvexMPC/ConvexMPC.hpp>
#include <iostream>
#include <Utils/IO/IOUtilities.hpp>

using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;

#ifdef _WIN32
typedef __int64 qp_int64;
#else
typedef long long qp_int64;
#endif //_WIN32

constexpr int k3Dim = 3;
constexpr double kGravity = 9.8;
constexpr double kMaxScale = 10;
constexpr double kMinScale = 0.1;
constexpr int ConvexMPC::kStateDim;

#define DCHECK_GT(a,b) assert((a)>(b))
#define DCHECK_EQ(a,b) assert((a)==(b))

Matrix3d ConvertRpyToRot(const Vector3d& rpy) {
    assert(rpy.size() == k3Dim);
    const AngleAxisd roll(rpy[0], Vector3d::UnitX());
    const AngleAxisd pitch(rpy[1], Vector3d::UnitY());
    const AngleAxisd yaw(rpy[2], Vector3d::UnitZ());
    Quaterniond q = yaw * pitch * roll;

    return q.matrix();
}

Matrix3d ConvertToSkewSymmetric(const Vector3d& vec) {
    Matrix3d skew_symm;
    skew_symm << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return skew_symm;
}

void CalculateAMat(const Vector3d& rpy, MatrixXd* a_mat_ptr) {
    // The transformation of angular velocity to roll pitch yaw rate. Caveat:
    // rpy rate is not a proper vector and does not follow the common vector
    // transformation dicted by the rotation matrix. Here we assume the input
    // rotation is in X->Y->Z order in the extrinsic/fixed frame, or z->y'->x''
    // order in the intrinsic frame.
    const double cos_yaw = cos(rpy[2]);
    const double sin_yaw = sin(rpy[2]);
    const double cos_pitch = cos(rpy[1]);
    const double tan_pitch = tan(rpy[1]);
    Matrix3d angular_velocity_to_rpy_rate;
    angular_velocity_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
        -sin_yaw, cos_yaw, 0, cos_yaw* tan_pitch, sin_yaw* tan_pitch, 1;

    MatrixXd& a_mat = *a_mat_ptr;
    a_mat.block<3, 3>(0, 6) = angular_velocity_to_rpy_rate;
    a_mat(3, 9) = 1;
    a_mat(4, 10) = 1;
    a_mat(5, 11) = 1;
    a_mat(11, 12) = 1;
}

void CalculateBMat(double inv_mass, const Matrix3d& inv_inertia,
    const MatrixXd& foot_positions, MatrixXd* b_mat_ptr) {
    // b_mat contains non_zero elements only in row 6:12.
    const int num_legs = foot_positions.rows();
    MatrixXd& b_mat = *b_mat_ptr;
    for (int i = 0; i < num_legs; ++i) {
        b_mat.block<k3Dim, k3Dim>(6, i * k3Dim) =
            inv_inertia * ConvertToSkewSymmetric(foot_positions.row(i));
        b_mat(9, i * k3Dim) = inv_mass;
        b_mat(10, i * k3Dim + 1) = inv_mass;
        b_mat(11, i * k3Dim + 2) = inv_mass;
    }
}

void CalculateExponentials(const MatrixXd& a_mat, const MatrixXd& b_mat,
    double timestep, MatrixXd* ab_mat_ptr,
    MatrixXd* a_exp_ptr, MatrixXd* b_exp_ptr) {
    const int state_dim = ConvexMPC::kStateDim;
    MatrixXd& ab_mat = *ab_mat_ptr;
    ab_mat.block<state_dim, state_dim>(0, 0) = a_mat * timestep;
    const int action_dim = b_mat.cols();
    ab_mat.block(0, state_dim, state_dim, action_dim) = b_mat * timestep;

    // This temporary is inevitable.
    MatrixXd ab_exp = ab_mat.exp();
    *a_exp_ptr = ab_exp.block<state_dim, state_dim>(0, 0);
    *b_exp_ptr = ab_exp.block(0, state_dim, state_dim, action_dim);
}

void CalculateQpMats(const MatrixXd& a_exp, const MatrixXd& b_exp,
    const MatrixXd& qp_weights_single,
    const MatrixXd& alpha_single, int horizon,
    MatrixXd* a_qp_ptr, MatrixXd* anb_aux_ptr,
    MatrixXd* b_qp_ptr, MatrixXd* p_mat_ptr) {
    const int state_dim = ConvexMPC::kStateDim;
    MatrixXd& a_qp = *a_qp_ptr;
    a_qp.block(0, 0, state_dim, state_dim) = a_exp;
    for (int i = 1; i < horizon - 1; ++i) {
        a_qp.block<state_dim, state_dim>(i * state_dim, 0) =
            a_exp * a_qp.block<state_dim, state_dim>((i - 1) * state_dim, 0);
    }

    const int action_dim = b_exp.cols();

    MatrixXd& anb_aux = *anb_aux_ptr;
    anb_aux.block(0, 0, state_dim, action_dim) = b_exp;
    for (int i = 1; i < horizon; ++i) {
        anb_aux.block(i * state_dim, 0, state_dim, action_dim) =
            a_exp * anb_aux.block((i - 1) * state_dim, 0, state_dim, action_dim);
    }

    MatrixXd& b_qp = *b_qp_ptr;
    for (int i = 0; i < horizon; ++i) {
        // Diagonal block.
        b_qp.block(i * state_dim, i * action_dim, state_dim, action_dim) = b_exp;
        // Off diagonal Diagonal block = A^(i - j - 1) * B_exp.
        for (int j = 0; j < i; ++j) {
            const int power = i - j;
            b_qp.block(i * state_dim, j * action_dim, state_dim, action_dim) =
                anb_aux.block(power * state_dim, 0, state_dim, action_dim);
        }
    }

    MatrixXd& p_mat = *p_mat_ptr;
    for (int i = horizon - 1; i >= 0; --i) {
        p_mat.block(i * action_dim, (horizon - 1) * action_dim, action_dim,
            action_dim) =
            anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
            .transpose() *
            qp_weights_single * b_exp;
        if (i != horizon - 1) {
            p_mat.block((horizon - 1) * action_dim, i * action_dim, action_dim,
                action_dim) =
                p_mat
                .block(i * action_dim, (horizon - 1) * action_dim, action_dim,
                    action_dim)
                .transpose();
        }
    }

    for (int i = horizon - 2; i >= 0; --i) {
        // Diagonal block.
        p_mat.block(i * action_dim, i * action_dim, action_dim, action_dim) =
            p_mat.block((i + 1) * action_dim, (i + 1) * action_dim, action_dim,
                action_dim) +
            anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
            .transpose() *
            qp_weights_single *
            anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim,
                action_dim);
        // Off diagonal block
        for (int j = i + 1; j < horizon - 1; ++j) {
            p_mat.block(i * action_dim, j * action_dim, action_dim, action_dim) =
                p_mat.block((i + 1) * action_dim, (j + 1) * action_dim, action_dim,
                    action_dim) +
                anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
                .transpose() *
                qp_weights_single *
                anb_aux.block((horizon - j - 1) * state_dim, 0, state_dim,
                    action_dim);
            p_mat.block(j * action_dim, i * action_dim, action_dim, action_dim) =
                p_mat.block(i * action_dim, j * action_dim, action_dim, action_dim)
                .transpose();
        }
    }

    p_mat *= 2.0;
    for (int i = 0; i < horizon; ++i) {
        p_mat.block(i * action_dim, i * action_dim, action_dim, action_dim) +=
            alpha_single;
    }
}

void UpdateConstraintsMatrix(VectorXd& friction_coeff,
    int horizon, int num_legs,
    MatrixXd* constraint_ptr) {
    const int constraint_dim = ConvexMPC::kConstraintDim;
    MatrixXd& constraint = *constraint_ptr;
    for (int i = 0; i < horizon * num_legs; ++i) {
        constraint.block<constraint_dim, k3Dim>(i * constraint_dim, i * k3Dim)
            << -1,
            0, friction_coeff[0], 1, 0, friction_coeff[1], 0, -1, friction_coeff[2],
            0, 1, friction_coeff[3], 0, 0, 1;
    }
}

void CalculateConstraintBounds(const MatrixXd& contact_state, double fz_max,
    double fz_min, double friction_coeff,
    int horizon, VectorXd* constraint_lb_ptr,
    VectorXd* constraint_ub_ptr) {
    const int constraint_dim = ConvexMPC::kConstraintDim;

    const int num_legs = contact_state.cols();

    VectorXd& constraint_lb = *constraint_lb_ptr;
    VectorXd& constraint_ub = *constraint_ub_ptr;
    for (int i = 0; i < horizon; ++i) {
        for (int j = 0; j < num_legs; ++j) {
            const int row = (i * num_legs + j) * constraint_dim;
            constraint_lb(row) = 0;
            constraint_lb(row + 1) = 0;
            constraint_lb(row + 2) = 0;
            constraint_lb(row + 3) = 0;
            constraint_lb(row + 4) = fz_min * contact_state(i, j);

            const double friction_ub =
                (friction_coeff + 1) * fz_max * contact_state(i, j);
            constraint_ub(row) = friction_ub;
            constraint_ub(row + 1) = friction_ub;
            constraint_ub(row + 2) = friction_ub;
            constraint_ub(row + 3) = friction_ub;
            constraint_ub(row + 4) = fz_max * contact_state(i, j);
        }
    }
}

double EstimateCoMHeightSimple(
    const MatrixXd& foot_positions_world,
    const VectorXi foot_contact_states) {
    int legs_in_contact = 0;
    double com_height = 0;
    const int z_dim = 2;
    for (int i = 0; i < foot_contact_states.size(); ++i) {
        if (foot_contact_states[i]) {
            com_height += foot_positions_world(i, z_dim);
            legs_in_contact += 1;
        }
    }

    // We don't support jumping in air for now.
    DCHECK_GT(legs_in_contact, 0);
    return abs(com_height / legs_in_contact);
}

MatrixXd AsBlockDiagonalMat(const VectorXd& qp_weights,
    int planning_horizon) {
    const Eigen::Map<const VectorXd> qp_weights_vec(qp_weights.data(),
        qp_weights.size());
    // Directly return the rhs will cause a TSAN failure, probably due to the
    // asDiagonal not reall copying the memory. Creates the temporary will ensure
    // copy on return.
    const MatrixXd qp_weights_mat =
        qp_weights_vec.replicate(planning_horizon, 1).asDiagonal();
    return qp_weights_mat;
}

ConvexMPC::ConvexMPC(double mass, const Eigen::VectorXd& inertia,
    int num_legs, int planning_horizon, double timestep,
    const Eigen::VectorXd& qp_weights, double alpha)
    : mass_(mass),
    inv_mass_(1 / mass),
    inertia_(inertia.data()),
    inv_inertia_(inertia_.inverse()),
    num_legs_(num_legs),
    planning_horizon_(planning_horizon),
    timestep_(timestep),
    qp_weights_(AsBlockDiagonalMat(qp_weights, planning_horizon)),
    qp_weights_single_(AsBlockDiagonalMat(qp_weights, 1)),
    alpha_(alpha* MatrixXd::Identity(num_legs* planning_horizon* k3Dim,
        num_legs* planning_horizon* k3Dim)),
    alpha_single_(alpha*
        MatrixXd::Identity(num_legs* k3Dim, num_legs* k3Dim)),
    action_dim_(num_legs* k3Dim),
    state_(kStateDim),
    desired_states_(kStateDim* planning_horizon),
    contact_states_(planning_horizon, num_legs),
    foot_positions_base_(num_legs, k3Dim),
    foot_positions_world_(num_legs, k3Dim),
    foot_friction_coeff_(num_legs_),
    a_mat_(kStateDim, kStateDim),
    b_mat_(kStateDim, action_dim_),
    ab_concatenated_(kStateDim + action_dim_, kStateDim + action_dim_),
    a_exp_(kStateDim, kStateDim),
    b_exp_(kStateDim, action_dim_),
    a_qp_(kStateDim* planning_horizon, kStateDim),
    b_qp_(kStateDim* planning_horizon, action_dim_* planning_horizon),
    p_mat_(num_legs* planning_horizon* k3Dim,
        num_legs* planning_horizon* k3Dim),
    q_vec_(num_legs* planning_horizon* k3Dim),
    anb_aux_(kStateDim* planning_horizon, action_dim_),
    constraint_(kConstraintDim* num_legs* planning_horizon,
        action_dim_* planning_horizon),
    constraint_lb_(kConstraintDim* num_legs* planning_horizon),
    constraint_ub_(kConstraintDim* num_legs* planning_horizon),
    qp_solution_(k3Dim* num_legs* planning_horizon),
    workspace_(0),
    initial_run_(true)
{
  assert(qp_weights.size() == kStateDim);
  // We assume the input inertia is a 3x3 matrix.
  assert(inertia.size() == k3Dim * k3Dim);
  state_.setZero();
  desired_states_.setZero();
  contact_states_.setZero();
  foot_positions_base_.setZero();
  foot_positions_world_.setZero();
  foot_friction_coeff_.setZero();
  a_mat_.setZero();
  b_mat_.setZero();
  ab_concatenated_.setZero();
  a_exp_.setZero();
  b_exp_.setZero();
  a_qp_.setZero();
  b_qp_.setZero();
  b_qp_transpose_.setZero();
  constraint_.setZero();
  constraint_lb_.setZero();
  constraint_ub_.setZero();
}

void ConvexMPC::ResetSolver() { initial_run_ = true; }

VectorXd ConvexMPC::ComputeContactForces(
      VectorXd com_position, // Eigen::VectorXd::Zero(1)
      VectorXd com_velocity, // Body Frame
      VectorXd com_roll_pitch_yaw, //
      VectorXd com_angular_velocity, // World Frame
      VectorXi foot_contact_states,
      VectorXd foot_positions_body_frame,
      VectorXd foot_friction_coeffs,
      VectorXd desired_com_position, // (0, 0, target_height)
      VectorXd desired_com_velocity, // input
      VectorXd desired_com_roll_pitch_yaw, // (0, 0, 0)
      VectorXd desired_com_angular_velocity, // input
      VectorXd &state_progression_) {

  // Test the inputs
  // myUtils::pretty_print(com_position, std::cout, "com_position");
  // myUtils::pretty_print(com_velocity, std::cout, "com_velocity");
  // myUtils::pretty_print(com_roll_pitch_yaw, std::cout, "com_roll_pitch_yaw");
  // myUtils::pretty_print(com_angular_velocity, std::cout, "com_angular_velocity");
  // myUtils::pretty_print(foot_positions_body_frame, std::cout, "foot_positions_body_frame");
  // myUtils::pretty_print(foot_friction_coeffs, std::cout, "foot_friction_coeffs");
  // myUtils::pretty_print(desired_com_position, std::cout, "desired_com_position");
  // myUtils::pretty_print(desired_com_velocity, std::cout, "desired_com_velocity");
  // myUtils::pretty_print(desired_com_roll_pitch_yaw, std::cout, "desired_com_roll_pitch_yaw");
  // myUtils::pretty_print(desired_com_angular_velocity, std::cout, "desired_com_angular_velocity");


  VectorXd error_result;

  // First we compute the foot positions in the world frame.
  DCHECK_EQ(com_roll_pitch_yaw.size(), k3Dim);
  const Quaterniond com_rotation =
      AngleAxisd(com_roll_pitch_yaw[0], Vector3d::UnitX()) *
      AngleAxisd(com_roll_pitch_yaw[1], Vector3d::UnitY()) *
      AngleAxisd(com_roll_pitch_yaw[2], Vector3d::UnitZ());
  // myUtils::pretty_print(com_rotation, std::cout, "com rotation quaternion");

  DCHECK_EQ(foot_positions_body_frame.size(), k3Dim * num_legs_);
  foot_positions_base_ = Eigen::Map<const MatrixXd>(
      foot_positions_body_frame.data(), k3Dim, num_legs_).transpose();
  for (int i = 0; i < num_legs_; ++i) {
      foot_positions_world_.row(i) = com_rotation * foot_positions_base_.row(i);
  }
  // myUtils::pretty_print(foot_positions_base_, std::cout, "foot_positions_base_");
  // myUtils::pretty_print(foot_positions_world_, std::cout, "foot_positions_world_");

  // Now we can estimate the body height using the world frame foot positions
  // and contact states. We use simple averges leg height here.
  DCHECK_EQ(foot_contact_states.size(), num_legs_);
  const double com_z =
      com_position.size() == k3Dim ? com_position[2] : EstimateCoMHeightSimple(foot_positions_world_, foot_contact_states);

  // In MPC planning we don't care about absolute position in the horizontal
  // plane.
  const double com_x = 0;
  const double com_y = 0;

  // Prepare the current and desired state vectors of length kStateDim *
  // planning_horizon.
  DCHECK_EQ(com_velocity.size(), k3Dim);
  DCHECK_EQ(com_angular_velocity.size(), k3Dim);
  state_ << com_roll_pitch_yaw[0], com_roll_pitch_yaw[1], com_roll_pitch_yaw[2],
            com_x, com_y, com_z, com_angular_velocity[0], com_angular_velocity[1],
            com_angular_velocity[2], com_velocity[0], com_velocity[1],
            com_velocity[2], -kGravity;
  // myUtils::pretty_print(state_, std::cout, "MPC state_");
  for (int i = 0; i < planning_horizon_; ++i) {
    desired_states_[i * kStateDim + 0] = desired_com_roll_pitch_yaw[0];
    desired_states_[i * kStateDim + 1] = desired_com_roll_pitch_yaw[1];
    desired_states_[i * kStateDim + 2] = 
            com_roll_pitch_yaw[2] +
            timestep_ * (i + 1) * desired_com_angular_velocity[2];

    desired_states_[i * kStateDim + 3] =
            timestep_ * (i + 1) * desired_com_velocity[0];
    desired_states_[i * kStateDim + 4] =
            timestep_ * (i + 1) * desired_com_velocity[1];
    desired_states_[i * kStateDim + 5] = desired_com_position[2];

    // Prefer to stablize roll and pitch.
    desired_states_[i * kStateDim + 6] = 0;
    desired_states_[i * kStateDim + 7] = 0;
    desired_states_[i * kStateDim + 8] = desired_com_angular_velocity[2];

    desired_states_[i * kStateDim + 9] = desired_com_velocity[0];
    desired_states_[i * kStateDim + 10] = desired_com_velocity[1];
    // Prefer to stablize the body height.
    desired_states_[i * kStateDim + 11] = 0;

    desired_states_[i * kStateDim + 12] = -kGravity;
  }
  // myUtils::pretty_print(desired_states_, std::cout, "MPC desired_states_");
  // std::cout << "desired_states_.size() = " << desired_states_.size() << std::endl;

  const Vector3d rpy(com_roll_pitch_yaw[0], com_roll_pitch_yaw[1],
                     com_roll_pitch_yaw[2]);
  // myUtils::pretty_print(rpy, std::cout, "rpy");

  CalculateAMat(rpy, &a_mat_);
  // myUtils::pretty_print(a_mat_, std::cout, "a_mat_");

  rotation_ = ConvertRpyToRot(rpy);
  const Matrix3d inv_inertia_world = rotation_ * inv_inertia_ * rotation_.transpose();

  CalculateBMat(inv_mass_, inv_inertia_world, foot_positions_world_, &b_mat_);
  // myUtils::pretty_print(b_mat_, std::cout, "b_mat_");

  CalculateExponentials(a_mat_, b_mat_, timestep_, &ab_concatenated_, &a_exp_,
                        &b_exp_);
  // myUtils::pretty_print(b_exp_, std::cout, "b_exp_");

  CalculateQpMats(a_exp_, b_exp_, qp_weights_single_, alpha_single_,
                  planning_horizon_, &a_qp_, &anb_aux_, &b_qp_, &p_mat_);
  // myUtils::pretty_print(a_qp_, std::cout, "a_qp_");
  // myUtils::pretty_print(anb_aux_, std::cout, "anb_aux_");
  // myUtils::pretty_print(b_qp_, std::cout, "b_qp_");
  // myUtils::pretty_print(p_mat_, std::cout, "p_mat_");

  const MatrixXd state_diff = a_qp_ * state_ - desired_states_;
  // myUtils::pretty_print(state_diff, std::cout, "state_diff");

  q_vec_ = 2 * b_qp_.transpose() * (qp_weights_ * state_diff);
  // myUtils::pretty_print(q_vec_, std::cout, "q_vec_");

  const VectorXd one_vec = VectorXd::Constant(planning_horizon_, 1.0);
  const VectorXd zero_vec = VectorXd::Zero(planning_horizon_);
  for (int j = 0; j < foot_contact_states.size(); ++j) {
    if (foot_contact_states[j]) {
        contact_states_.col(j) = one_vec;
    } else {
        contact_states_.col(j) = zero_vec;
    }
  }

  CalculateConstraintBounds(contact_states_, mass_ * kGravity * kMaxScale,
                            mass_ * kGravity * kMinScale,
                            foot_friction_coeffs[0], planning_horizon_,
                            &constraint_lb_, &constraint_ub_);
  // myUtils::pretty_print(constraint_ub_, std::cout, "constraint bounds");

  UpdateConstraintsMatrix(foot_friction_coeffs, planning_horizon_, num_legs_,
                          &constraint_);
  // myUtils::pretty_print(constraint_, std::cout, "constraint matrix");
  foot_friction_coeff_ << foot_friction_coeffs[0], foot_friction_coeffs[1],
                          foot_friction_coeffs[2], foot_friction_coeffs[3];


  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int64> objective_matrix = p_mat_.sparseView();
  Eigen::VectorXd objective_vector = q_vec_;
  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int64> constraint_matrix = constraint_.sparseView();

  // AMBIGUOUS myUtils::pretty_print(objective_matrix, std::cout, "objective_matrix");
  // myUtils::pretty_print(objective_vector, std::cout, "objective vector");
  // AMBIGUOUS myUtils::pretty_print(constraint_matrix, std::cout, "constraint_matrix");

  int num_variables = constraint_.cols();
  int num_constraints = constraint_.rows();

  ::OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.warm_start = true;
  settings.polish = true;
  settings.adaptive_rho_interval = 25;
  settings.eps_abs = 1e-3;
  settings.eps_rel = 1e-3;

  assert(p_mat_.cols()== num_variables);
  assert(p_mat_.rows()== num_variables);
  assert(q_vec_.size()== num_variables);
  assert(constraint_lb_.size() == num_constraints);
  assert(constraint_ub_.size() == num_constraints);

  VectorXd clipped_lower_bounds = constraint_lb_.cwiseMax(-OSQP_INFTY);
  VectorXd clipped_upper_bounds = constraint_ub_.cwiseMin(OSQP_INFTY);

  ::OSQPData data;
  data.n = num_variables;
  data.m = num_constraints;

  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int64>
        objective_matrix_upper_triangle =
        objective_matrix.triangularView<Eigen::Upper>();

  ::csc osqp_objective_matrix = {
        objective_matrix_upper_triangle.outerIndexPtr()[num_variables],
        num_variables,
        num_variables,
        const_cast<qp_int64*>(objective_matrix_upper_triangle.outerIndexPtr()),
        const_cast<qp_int64*>(objective_matrix_upper_triangle.innerIndexPtr()),
        const_cast<double*>(objective_matrix_upper_triangle.valuePtr()),
        -1 };
  data.P = &osqp_objective_matrix;
  for(int i=0; i<objective_matrix_upper_triangle.size(); ++i){}
  // std::cout << "objective_matrix_upper_triangle.size() = " << objective_matrix_upper_triangle.size() << std::endl;
  // std::cout << "rows = " << num_constraints << std::endl;
  // std::cout << "columns = " << num_variables << std::endl;

  ::csc osqp_constraint_matrix = {
      constraint_matrix.outerIndexPtr()[num_variables],
      num_constraints,
      num_variables,
      const_cast<qp_int64*>(constraint_matrix.outerIndexPtr()),
      const_cast<qp_int64*>(constraint_matrix.innerIndexPtr()),
      const_cast<double*>(constraint_matrix.valuePtr()),
      -1 };
  data.A = &osqp_constraint_matrix;

  data.q = const_cast<double*>(objective_vector.data());
  data.l = clipped_lower_bounds.data();
  data.u = clipped_upper_bounds.data();

  const int return_code = 0;

  if (workspace_==0) {
      osqp_setup(&workspace_, &data, &settings);
      initial_run_ = false;
  } else {

      UpdateConstraintsMatrix(foot_friction_coeffs, planning_horizon_,
                              num_legs_, &constraint_);
      foot_friction_coeff_ << foot_friction_coeffs[0], foot_friction_coeffs[1],
                              foot_friction_coeffs[2], foot_friction_coeffs[3];

      c_int nnzP = objective_matrix_upper_triangle.nonZeros();

      c_int nnzA = constraint_matrix.nonZeros();

      int return_code = osqp_update_P_A(
            workspace_, objective_matrix_upper_triangle.valuePtr(), OSQP_NULL, nnzP,
            constraint_matrix.valuePtr(), OSQP_NULL, nnzA);

      return_code =
            osqp_update_lin_cost(workspace_, objective_vector.data());


      return_code = osqp_update_bounds(
            workspace_, clipped_lower_bounds.data(), clipped_upper_bounds.data());
  }

  if (osqp_solve(workspace_) != 0) {
    if (osqp_is_interrupted()) {
        return error_result;
    }
  }

  Map<VectorXd> solution(qp_solution_.data(), qp_solution_.size());

  if (workspace_->info->status_val== OSQP_SOLVED) {
      solution = -Map<const VectorXd>(workspace_->solution->x, workspace_->data->n);
  } else {
      //LOG(WARNING) << "QP does not converge";
      std::cout << "QP Does not Converge" << std::endl;
      return error_result;
  }
  // myUtils::pretty_print(qp_solution_, std::cout, "qp solution");
  // std::cout << "a_qp_ rows, cols = " << a_qp_.rows() << ", " << a_qp_.cols() << std::endl;
  // std::cout << "b_qp_ rows, cols = " << b_qp_.rows() << ", " << b_qp_.cols() << std::endl;
  state_progression_ = a_qp_ * state_ + b_qp_ * qp_solution_;
  // myUtils::pretty_print(state_progression_, std::cout, "state progression");
  return qp_solution_;

}
