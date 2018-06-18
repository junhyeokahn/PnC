#ifndef JSPACE_PSEUDO_INVERSE_HPP
#define JSPACE_PSEUDO_INVERSE_HPP

#include <Eigen/Dense>

namespace myUtils {
  void pseudoInverse(Eigen::MatrixXd const & matrix,
                     double sigmaThreshold,
                     Eigen::MatrixXd & invMatrix,
                     Eigen::VectorXd * opt_sigmaOut = 0);
}

#endif
