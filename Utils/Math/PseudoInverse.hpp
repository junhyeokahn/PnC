#ifndef JSPACE_PSEUDO_INVERSE_HPP
#define JSPACE_PSEUDO_INVERSE_HPP

#include <Eigen/Dense>

namespace myUtils {
void pseudo_inverse(Eigen::MatrixXd const &matrix, double sigmaThreshold,
                    Eigen::MatrixXd &invMatrix,
                    Eigen::VectorXd *opt_sigmaOut = 0);

Eigen::MatrixXd get_null_space(const Eigen::MatrixXd &J,
                               const double threshold = 0.00001);

Eigen::MatrixXd weighted_pseduo_inverse(const Eigen::MatrixXd &J,
                                        const Eigen::MatrixXd &W,
                                        const double sigma_threshold = 0.0001);
} // namespace myUtils

#endif
