#include <Utils/ThirdPartyUtilities.hpp>
#include <drake/math/eigen_sparse_triplet.h>

namespace myUtils
{
    void collectNonZeroIdxAndValue( const Eigen::MatrixXd A,
            Eigen::VectorXi & rows,
            Eigen::VectorXi & cols,
            Eigen::VectorXd & vals) {
        std::vector<Eigen::Index> row_indices;
        std::vector<Eigen::Index> col_indices;
        std::vector<double> val;
        drake::math::SparseMatrixToRowColumnValueVectors(A, row_indices, col_indices, val);
        std::vector<int> non_zero_row_indices;
        std::vector<int> non_zero_col_indices;
        std::vector<double> non_zero_val;
        for (int i = 0; i < row_indices.size(); ++i) {
            if (!(val[i] > -0.00001 && val[i] < 0.00001)) {
                non_zero_row_indices.push_back(row_indices[i]);
                non_zero_col_indices.push_back(col_indices[i]);
                non_zero_val.push_back(val[i]);
            }
        }
        rows.resize(non_zero_row_indices.size());
        cols.resize(non_zero_row_indices.size());
        vals.resize(non_zero_row_indices.size());
        for (int i = 0; i < non_zero_row_indices.size(); ++i) {
            rows[i] = non_zero_row_indices[i];
            cols[i] = non_zero_col_indices[i];
            vals[i] = non_zero_val[i];
        }
    }

} /* myUtils */ 
