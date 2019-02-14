#include <Utils/Math/MathUtilities.hpp>
#include <cmath>

namespace myUtils {
Eigen::MatrixXd hStack(const Eigen::MatrixXd a, const Eigen::MatrixXd b) {
    if (a.rows() != b.rows()) {
        std::cout << "[hStack] Matrix Size is Wrong" << std::endl;
        exit(0);
    }

    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
    ab << a, b;
    return ab;
}

Eigen::MatrixXd vStack(const Eigen::MatrixXd a, const Eigen::MatrixXd b) {
    if (a.cols() != b.cols()) {
        std::cout << "[vStack] Matrix Size is Wrong" << std::endl;
        exit(0);
    }
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
    ab << a, b;
    return ab;
}

Eigen::MatrixXd vStack(const Eigen::VectorXd a, const Eigen::VectorXd b) {
    if (a.size() != b.size()) {
        std::cout << "[vStack] Vector Size is Wrong" << std::endl;
        exit(0);
    }
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.size(), 2);
    ab << a, b;
    return ab;
}

Eigen::MatrixXd deleteRow(const Eigen::MatrixXd& a_, int row_) {
    Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(a_.rows() - 1, a_.cols());
    ret.block(0, 0, row_, a_.cols()) = a_.block(0, 0, row_, a_.cols());
    ret.block(row_, 0, ret.rows() - row_, a_.cols()) =
        a_.block(row_ + 1, 0, ret.rows() - row_, a_.cols());
    return ret;
}

double bind_half_pi(double ang) {
    if (ang > M_PI / 2) {
        return ang - M_PI;
    }
    if (ang < -M_PI / 2) {
        return ang + M_PI;
    }
    return ang;
}

bool isInBoundingBox(const Eigen::VectorXd& lb, const Eigen::VectorXd& val,
                     const Eigen::VectorXd& ub) {
    int n = lb.size();
    bool ret(true);
    for (int i = 0; i < n; ++i) {
        if (lb[i] <= val[i] && val[i] <= ub[i]) {
        } else {
            std::cout << i << " th : lb = " << lb[i] << " val = " << val[i]
                      << " ub = " << ub[i] << std::endl;
            ret = false;
        }
    }
    return ret;
}

Eigen::VectorXd eulerIntegration(const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& xdot, double dt) {
    Eigen::VectorXd ret = x;
    ret += xdot * dt;
    return ret;
}

Eigen::VectorXd doubleIntegration(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& alpha,
                                  const Eigen::VectorXd& alphad, double dt) {
    Eigen::VectorXd ret = q;
    ret += alpha * dt + alphad * dt * dt * 0.5;
    return ret;
}

}  // namespace myUtils
