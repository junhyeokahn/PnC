#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "fusion.h"
#include <cassert>

#include <Eigen/Dense>

using namespace mosek::fusion;
using namespace monty;

namespace LownerJohnEllipsoid
{
    std::shared_ptr<ndarray<int, 1>> range(int start, int stop); int pow2(int n);
    void geometric_mean(Model::t M, Variable::t x, Variable::t t);
    void det_rootn(Model::t M, int n, Variable::t X, Variable::t t);

    std::pair<std::shared_ptr<ndarray<double,1>>,std::shared_ptr<ndarray<double,1> > >
        lownerjohn_inner
        ( std::shared_ptr<ndarray<double,2>> A,
          std::shared_ptr<ndarray<double,1>> b );

    /*
     * (Qx - c)'(Qx - c) = 1
     * In order to write with |x - b|_{P} = 1,
     * P = Q.transpose()*Q, b = Q.inverse()*c
     */
    std::pair<std::shared_ptr<ndarray<double,1>>,std::shared_ptr<ndarray<double,1 > > >
        lownerjohn_outer(std::shared_ptr<ndarray<double,2>> x);

    std::ostream & operator<<(std::ostream & os, ndarray<double,1> & a);
} /* LownerJohnEllipsoid */

 /*
  * Class for Computing LownerJohnEllipsoid with Points and Normal Vector in
  * Global Coordinate
  */
class LJEllipsoidWithPointsAndNormal
{
public:
    LJEllipsoidWithPointsAndNormal (std::vector<Eigen::VectorXd> pointsList_,
                                    Eigen::VectorXd normalVector_);
    virtual ~LJEllipsoidWithPointsAndNormal ();

    /*
     * Return Spectral Matrix P and Center b
     * Such that (x - b)'P(x - b) = 1
     * Note that P, b is in local coordinate
     */
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> computeLJEllipsoid();
    double getNullDimVal();
    // local coordinate w.r.t global
    Eigen::MatrixXd getSO();

private:
    std::vector<Eigen::VectorXd> mPointsList;
    Eigen::VectorXd mNormalVector;
    int mDim;
    int mNumPoints;
    double mNullDimVal;
    Eigen::MatrixXd mSO;
};
