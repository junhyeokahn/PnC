#include <Utils/Math/MathUtilities.hpp>
#include <cassert>
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
             myUtils::color_print(myColor::BoldMagneta, "Is not BoundingBox");
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

double CropValue(double value, double min, double max, std::string source) {
    assert(min < max);
    if (value > max) {
        printf("%s: %f is cropped to %f.\n", source.c_str(), value, max);
        value = max;
    }
    if (value < min) {
        printf("%s: %f is cropped to %f.\n", source.c_str(), value, min);
        value = min;
    }
    return value;
}

Eigen::VectorXd CropVector(Eigen::VectorXd value, Eigen::VectorXd min,
                           Eigen::VectorXd max, std::string source) {
    assert(value.size() = min.size());
    assert(value.size() = max.size());
    int n_data = value.size();

    for (int i = 0; i < n_data; ++i) {
        if (value[i] > max[i]) {
            // printf("%s(%d): %f is cropped to %f\n", source.c_str(), i,
            // value[i], max[i]);
            value[i] = max[i];
        }
        if (value[i] < min[i]) {
            // printf("%s(%d): %f is cropped to %f\n", source.c_str(), i,
            // value[i], min[i]);
            value[i] = min[i];
        }
    }
    return value;
}

Eigen::MatrixXd CropMatrix(Eigen::MatrixXd value, Eigen::MatrixXd min,
                           Eigen::MatrixXd max, std::string source) {
    assert(value.cols() = min.cols());
    assert(value.rows() = min.rows());
    assert(value.cols() = max.cols());
    assert(value.rows() = max.rows());

    int n_row = value.rows();
    int n_cols = value.cols();

    for (int row_idx = 0; row_idx < n_row; ++row_idx) {
        for (int col_idx = 0; col_idx < n_cols; ++col_idx) {
            if (value(row_idx, col_idx) < min(row_idx, col_idx)) {
                // printf("%s(%d, %d): %f is cropped to %f\n", source.c_str(),
                // row_idx, col_idx, value(row_idx, col_idx), min(row_idx,
                // col_idx));
                value(row_idx, col_idx) = min(row_idx, col_idx);
            }
            if (value(row_idx, col_idx) > max(row_idx, col_idx)) {
                // printf("%s(%d, %d): %f is cropped to %f\n", source.c_str(),
                // row_idx, col_idx, value(row_idx, col_idx), max(row_idx,
                // col_idx));
                value(row_idx, col_idx) = max(row_idx, col_idx);
            }
        }
    }
    return value;
}

}  // namespace myUtils
