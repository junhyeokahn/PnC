#include "Geometry/Polytope/Polytope.h"

Polytope::Polytope()
    : matPtr_(nullptr)
    , polytope_(nullptr)
{
    dd_set_global_constants();
}

Polytope::~Polytope()
{
    if (matPtr_ != nullptr)
        dd_FreeMatrix(matPtr_);
    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);
    dd_free_global_constants();
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polytope::vrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    isFromGenerators_ = false;
    if (!hvrep(A, b))
        throw std::runtime_error("Bad conversion from hrep to vrep.");

    return vrep();
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polytope::hrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    isFromGenerators_ = true;
    if (!hvrep(A, b))
        throw std::runtime_error("Bad conversion from vrep to hrep.");
    return hrep();
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polytope::vrep()
{
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    return ddfMatrix2EigenMatrix(mat);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polytope::hrep()
{
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    return ddfMatrix2EigenMatrix(mat);
}

void Polytope::printVrep()
{
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    dd_WriteMatrix(stdout, mat);
}

void Polytope::printHrep()
{
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    dd_WriteMatrix(stdout, mat);
}

/**
 * Private functions
 */

bool Polytope::hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    Eigen::MatrixXd cMat = concatenateMatrix(A, b);
    return doubleDescription(cMat);
}

void Polytope::initializeMatrixPtr(Eigen::Matrix<int, 1, 1>::Index rows, Eigen::Matrix<int, 1, 1>::Index cols)
{
    if (matPtr_ != nullptr)
        dd_FreeMatrix(matPtr_);
    matPtr_ = dd_CreateMatrix(rows, cols);
    matPtr_->representation = (isFromGenerators_ ? dd_Generator : dd_Inequality);
}

bool Polytope::doubleDescription(const Eigen::MatrixXd& matrix)
{
    initializeMatrixPtr(matrix.rows(), matrix.cols());

    for (auto row = 0; row < matrix.rows(); ++row)
        for (auto col = 0; col < matrix.cols(); ++col)
            matPtr_->matrix[row][col][0] = matrix(row, col);

    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);
    polytope_ = dd_DDMatrix2Poly(matPtr_, &err_);
    return (err_ == dd_NoError) ? true : false;
}

Eigen::MatrixXd Polytope::concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    double sign = (isFromGenerators_ ? 1 : -1);
    Eigen::MatrixXd mat(A.rows(), A.cols() + 1);
    mat.col(0) = b;
    mat.block(0, 1, A.rows(), A.cols()).noalias() = sign * A;
    return mat;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polytope::ddfMatrix2EigenMatrix(dd_MatrixPtr mat)
{
    double sign = (isFromGenerators_ ? -1 : 1);
    auto rows = mat->rowsize;
    auto cols = mat->colsize;
    Eigen::MatrixXd mOut(rows, cols - 1);
    Eigen::VectorXd vOut(rows);
    for (auto row = 0; row < rows; ++row) {
        vOut(row) = mat->matrix[row][0][0];
        for (auto col = 1; col < cols; ++col)
            mOut(row, col - 1) = sign * mat->matrix[row][col][0];
    }

    return std::make_pair(mOut, vOut);
}
