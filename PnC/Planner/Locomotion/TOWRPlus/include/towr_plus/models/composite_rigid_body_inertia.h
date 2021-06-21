/******************************************************************************
Written by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#pragma once

#include "util/util.hpp"

class MLPModel;

namespace towr_plus {

class CompositeRigidBodyInertia {
public:
  CompositeRigidBodyInertia(int n_input, int dim_per_input);

  virtual ~CompositeRigidBodyInertia();

  virtual Eigen::MatrixXd ComputeInertia(const Eigen::Vector3d &base_pos,
                                         const Eigen::Vector3d &lf_pos,
                                         const Eigen::Vector3d &rf_pos) = 0;

  virtual Eigen::MatrixXd
  ComputeDerivativeWrtInput(const Eigen::Vector3d &base_pos,
                            const Eigen::Vector3d &lf_pos,
                            const Eigen::Vector3d &rf_pos) = 0;

protected:
  // Create 3x3 inertia matrix
  Eigen::MatrixXd _inertia_from_one_hot_vector(const Eigen::VectorXd &vec);
  // Use this for filling function args
  void _fill_double_array(const Eigen::MatrixXd &mat, double **x);
  // Use this for filling jacobian function args
  void _fill_double_array(const Eigen::MatrixXd &mat1,
                          const Eigen::MatrixXd &mat2, double **x);
  // Use this for parsing output
  // Assume x[1][mat.cols()*mat.rows()]
  void _fill_matrix(double **x, Eigen::MatrixXd &mat, int block_row,
                    int block_col);

  int n_input_;
  int dim_per_input_;
  int n_output_;
  int dim_per_output_;
};

} /* namespace towr_plus */
