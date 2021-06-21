#include "Utils/Math/PseudoInverse.hpp"
#include <Eigen/LU>
#include <Eigen/SVD>
#include <stdio.h>
using namespace std;

namespace myUtils {

void pseudo_inverse(Eigen::MatrixXd const &matrix, double sigmaThreshold,
                    Eigen::MatrixXd &invMatrix, Eigen::VectorXd *opt_sigmaOut) {

  if ((1 == matrix.rows()) && (1 == matrix.cols())) {
    // workaround for Eigen2
    invMatrix.resize(1, 1);
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    if (opt_sigmaOut) {
      opt_sigmaOut->resize(1);
      opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
    }
    return;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU |
                                                    Eigen::ComputeThinV);
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());
  Eigen::MatrixXd invS;
  invS = Eigen::MatrixXd::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {
      // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
      // printf("sigular value is too small: %f\n",
      // svd.singularValues().coeff(ii));
    }
  }
  invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
  if (opt_sigmaOut) {
    *opt_sigmaOut = svd.singularValues();
  }
}

Eigen::MatrixXd getNullSpace(const Eigen::MatrixXd &J, const double threshold) {

  Eigen::MatrixXd ret(J.cols(), J.cols());
  Eigen::MatrixXd J_pinv;
  myUtils::pseudo_inverse(J, threshold, J_pinv);
  ret = Eigen::MatrixXd::Identity(J.cols(), J.cols()) - J_pinv * J;
  return ret;
}

Eigen::MatrixXd weighted_pseduo_inverse(const Eigen::MatrixXd &J,
                                        const Eigen::MatrixXd &W,
                                        const double sigma_threshold) {
  Eigen::MatrixXd Jinv;
  Eigen::MatrixXd lambda(J * W * J.transpose());
  Eigen::MatrixXd lambda_inv;
  myUtils::pseudo_inverse(lambda, sigma_threshold, lambda_inv);
  Jinv = W * J.transpose() * lambda_inv;
  return Jinv;
}

} // namespace myUtils
