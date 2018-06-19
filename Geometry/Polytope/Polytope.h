#include <Eigen/Core>
#include <cdd/src/setoper.h> // Must be included before cdd.h (wtf)
#include <cdd/src/cdd.h>
#include <utility>

class Polytope {
public:
    /* Default constructor that set cdd global constants. */
    Polytope();
    /* Free the pointers and unset the cdd global constants. */
    ~Polytope();

    /* Treat the inputs as a H-representation and compute its V-representation.
     * H-polyhedron is such that \f$ Ax \leq b \f$.
     * \param A The matrix part of the representation of the polyhedron.
     * \param b The vector part of the representation of the polyhedron.
     * \return The V-representation of the polyhedron.
     */
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> vrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    /* Treat the inputs as a V-representation and compute its H-representation
     * V-polyhedron is such that \f$ A = [v^T r^T]^T, b=[1^T 0^T]^T \f$
     * with A composed of \f$ v \f$, the vertices, \f$ r \f$, the rays
     * and b is a vector which is 1 for vertices and 0 for rays.
     * \param A The matrix part of the representation of the polyhedron.
     * \param b The vector part of the representation of the polyhedron.
     * \return The H-representation of the polyhedron.
     */
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> hrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    /* Get the V-representation of the polyhedron
     * V-polyhedron is such that \f$ A = [v^T r^T]^T, b=[1^T 0^T]^T \f$
     * with A composed of \f$ v \f$, the vertices, \f$ r \f$, the rays
     * and b is a vector which is 1 for vertices and 0 for rays.
     * \return Pair of vertices and rays matrix and identification vector of vertices and rays for the V-representation
     */
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> vrep();
    /* Get the H-representation of the polyhedron
     * H-polyhedron is such that \f$ Ax \leq b \f$.
     * \return Pair of inequality matrix and inequality vector for the H-representation
     */
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> hrep();

    /* Print the H-representation of the polyhedron */
    void printHrep();
    /* Print the V-representation of the polyhedron */
    void printVrep();

private:
    bool hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    void initializeMatrixPtr(Eigen::Matrix<int, 1, 1>::Index rows, Eigen::Matrix<int, 1, 1>::Index cols);
    bool doubleDescription(const Eigen::MatrixXd& matrix);
    Eigen::MatrixXd concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> ddfMatrix2EigenMatrix(dd_MatrixPtr mat);

private:
    bool isFromGenerators_;
    dd_MatrixPtr matPtr_;
    dd_PolyhedraPtr polytope_;
    dd_ErrorType err_;
};
