#include <towr_plus/models/composite_rigid_body_inertia.h>

namespace towr_plus {

CompositeRigidBodyInertia::CompositeRigidBodyInertia(int n_input,
                                                     int dim_per_input) {
  n_input_ = n_input;
  dim_per_input_ = dim_per_input;
  n_output_ = 1;
  dim_per_output_ = 6;
}

CompositeRigidBodyInertia::~CompositeRigidBodyInertia() {}

Eigen::MatrixXd CompositeRigidBodyInertia::_inertia_from_one_hot_vector(
    const Eigen::VectorXd &vec) {
  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(3, 3);
  ret(0, 0) = vec[0];
  ret(1, 1) = vec[1];
  ret(2, 2) = vec[2];

  ret(0, 1) = vec[3];
  ret(1, 0) = vec[3];
  ret(0, 2) = vec[4];
  ret(2, 0) = vec[4];
  ret(1, 2) = vec[5];
  ret(2, 1) = vec[5];

  return ret;
}

void CompositeRigidBodyInertia::_fill_double_array(const Eigen::MatrixXd &mat,
                                                   double **x) {
  for (int row = 0; row < mat.rows(); ++row) {
    for (int col = 0; col < mat.cols(); ++col) {
      x[col][row] = mat(col, row);
    }
  }
}

void CompositeRigidBodyInertia::_fill_double_array(const Eigen::MatrixXd &mat1,
                                                   const Eigen::MatrixXd &mat2,
                                                   double **x) {
  for (int row = 0; row < mat1.rows(); ++row) {
    for (int col = 0; col < mat1.cols(); ++col) {
      x[row][col] = mat1(row, col);
    }
  }
  for (int row = 0; row < mat2.rows(); ++row) {
    for (int col = 0; col < mat2.cols(); ++col) {
      x[mat1.rows() + row][col] = mat2(row, col);
    }
  }
}

void CompositeRigidBodyInertia::_fill_matrix(double **x, Eigen::MatrixXd &mat,
                                             int block_row, int block_col) {
  int n_var(mat.cols() / block_col);
  int idx(0);
  for (int block_id = 0; block_id < n_var; ++block_id) {
    for (int col = 0; col < block_col; ++col) {
      for (int row = 0; row < block_row; ++row) {
        mat(row, block_id * block_col + col) = x[0][idx];
        idx += 1;
      }
    }
  }
}

} /* namespace towr_plus */
