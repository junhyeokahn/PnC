#include <Eigen/Dense>

namespace myUtils
{
     void collectNonZeroIdxAndValue( const Eigen::MatrixXd A,
                                          Eigen::VectorXi & rows,
                                          Eigen::VectorXi & cols,
                                          Eigen::VectorXd & vals);
} /* myUtils */ 
