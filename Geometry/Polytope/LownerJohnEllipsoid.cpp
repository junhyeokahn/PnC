/*
Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.

File:      lownerjohn_ellipsoid.cc

Purpose: 
Computes the Lowner-John inner and outer ellipsoidal 
approximations of a polytope.


The inner ellipsoidal approximation to a polytope 

S = { x \in R^n | Ax < b }.

maximizes the volume of the inscribed ellipsoid,

{ x | x = C*u + d, || u ||_2 <= 1 }.

The volume is proportional to det(C)^(1/n), so the
problem can be solved as 

maximize         t
subject to       t       <= det(C)^(1/n)
|| C*ai ||_2 <= bi - ai^T * d,  i=1,...,m
C is PSD

which is equivalent to a mixed conic quadratic and semidefinite
programming problem.


The outer ellipsoidal approximation to a polytope given 
as the convex hull of a set of points

S = conv{ x1, x2, ... , xm }

minimizes the volume of the enclosing ellipsoid,

{ x | || P*(x-c) ||_2 <= 1 }

The volume is proportional to det(P)^{-1/n}, so the problem can
be solved as

minimize         t
subject to       t       >= det(P)^(-1/n)
|| P*xi + c ||_2 <= 1,  i=1,...,m
P is PSD.

References:
[1] "Lectures on Modern Optimization", Ben-Tal and Nemirovski, 2000. 
*/

//#include <string>
//#include <iostream>
//#include <iomanip>
//#include <cmath>
//#include "fusion.h"
//#include <cassert>
#include "LownerJohnEllipsoid.hpp"

using namespace mosek::fusion;
using namespace monty;

namespace LownerJohnEllipsoid
{
    std::shared_ptr<ndarray<int,1>> range (int start, int stop)
    { 
        if (start < stop)
            return std::shared_ptr<ndarray<int,1>>(new ndarray<int,1>(shape_t<1>(stop-start),iterable(range_t<int>(start,stop))));
        else
            return new_array_ptr<int,1>(0);
    }


    int pow2(int n) { return (int) (1 << n); }
    /** 
Purpose: Models the convex set 

S = { (x, t) \in R^n x R | x >= 0, t <= (x1 * x2 * ... *xn)^(1/n) }.

as the intersection of rotated quadratic cones and affine hyperplanes,
see [1, p. 105].  This set can be interpreted as the hypograph of the 
geometric mean of x.

We illustrate the modeling procedure using the following example.
Suppose we have 

t <= (x1 * x2 * x3)^(1/3)

for some t >= 0, x >= 0. We rewrite it as

t^4 <= x1 * x2 * x3 * x4,   x4 = t

which is equivalent to (see [1])

x11^2 <= 2*x1*x2,   x12^2 <= 2*x3*x4,

x21^2 <= 2*x11*x12,

sqrt(8)*x21 = t, x4 = t.

References:
[1] "Lectures on Modern Optimization", Ben-Tal and Nemirovski, 2000. 
*/ 
    void geometric_mean(Model::t M, Variable::t x, Variable::t t)
    {
        int n = (int) x->size();
        int l = (int)std::ceil(std::log(n)/std::log(2));
        int m = pow2(l) - n;

        Variable::t x0 =
            m == 0 ? x : Var::vstack(x, M->variable(m, Domain::greaterThan(0.0)));

        Variable::t z = x0;

        for (int i = 0; i < l-1; ++i)
        {
            Variable::t xi = M->variable(pow2(l-i-1), Domain::greaterThan(0.0));
            for (int k = 0; k < pow2(l-i-1); ++k)
                M->constraint(Var::vstack(z->index(2*k),z->index(2*k+1),xi->index(k)),
                        Domain::inRotatedQCone());
            z = xi;
        }

        Variable::t t0 = M->variable(1, Domain::greaterThan(0.0));
        M->constraint(Var::vstack(z, t0), Domain::inRotatedQCone());

        M->constraint(Expr::sub(Expr::mul(std::pow(2,0.5*l),t),t0), Domain::equalsTo(0.0));

        for (int i = pow2(l-m); i < pow2(l); ++i)
            M->constraint(Expr::sub(x0->index(i), t), Domain::equalsTo(0.0));
    }

    /**
Purpose: Models the hypograph of the n-th power of the
determinant of a positive definite matrix.

The convex set (a hypograph)

C = { (X, t) \in S^n_+ x R |  t <= det(X)^{1/n} },

can be modeled as the intersection of a semidefinite cone

[ X, Z; Z^T Diag(Z) ] >= 0  

and a number of rotated quadratic cones and affine hyperplanes,

t <= (Z11*Z22*...*Znn)^{1/n}  (see geometric_mean).

References:
[1] "Lectures on Modern Optimization", Ben-Tal and Nemirovski, 2000. 
*/
    void det_rootn(Model::t M, int n, Variable::t X, Variable::t t)
    {
        //int n = X->get_shape()->dim(0);

        // Setup variables
        Variable::t Y = M->variable(Domain::inPSDCone(2*n));

        // Setup Y = [X, Z; Z^T diag(Z)] 
        Variable::t Y11 = Y->slice(new_array_ptr<int,1>({0, 0}), new_array_ptr<int,1>({n, n}));
        Variable::t Y21 = Y->slice(new_array_ptr<int,1>({n, 0}), new_array_ptr<int,1>({2*n, n}));
        Variable::t Y22 = Y->slice(new_array_ptr<int,1>({n, n}), new_array_ptr<int,1>({2*n, 2*n}));

        M->constraint( Expr::sub(Expr::mulElm( Matrix::eye(n) ,Y21), Y22), Domain::equalsTo(0.0));
        M->constraint( Expr::sub(X, Y11), Domain::equalsTo(0.0) );

        // t^n <= (Z11*Z22*...*Znn)

        geometric_mean(M, Y22->diag(), t);
    }

    std::pair<std::shared_ptr<ndarray<double,1>>,std::shared_ptr<ndarray<double,1>>>
        lownerjohn_inner
        ( std::shared_ptr<ndarray<double,2>> A, 
          std::shared_ptr<ndarray<double,1>> b)
        {
            Model::t M = new Model("lownerjohn_inner"); auto _M = finally([&]() { M->dispose(); });
            int m = A->size(0);
            int n = A->size(1);

            // Setup variables
            Variable::t t = M->variable("t", 1, Domain::greaterThan(0.0));   
            Variable::t C = M->variable("C", Set::make(n,n), Domain::unbounded());
            Variable::t d = M->variable("d", n, Domain::unbounded());        

            // qc_i : (bi - ai^T*d, C*ai) \in Q 
            for (int i = 0; i < m; ++i)
            {
                std::shared_ptr<ndarray<double,1>> ai = new_array_ptr<double,1>({(*A)(i,0),(*A)(i,1)});
                M->constraint(Expr::vstack(Expr::sub((*b)[i],Expr::dot(ai,d)), Expr::mul(C,ai)), Domain::inQCone() );
            }

            // t <= det(C)^{1/n}
            //model_utils.det_rootn(M, C, t);
            det_rootn(M,n,C,t);

            // Objective: Maximize t
            M->objective(ObjectiveSense::Maximize, t);
            M->solve();

            return std::make_pair(C->level(),d->level());
        }

    std::pair<std::shared_ptr<ndarray<double,1>>,std::shared_ptr<ndarray<double,1>>>
        lownerjohn_outer(std::shared_ptr<ndarray<double,2>> x)
        {
            Model::t M = new Model("lownerjohn_outer");
            int m = x->size(0);
            int n = x->size(1);

            // Setup variables
            Variable::t t = M->variable("t", 1, Domain::greaterThan(0.0));
            Variable::t P = M->variable("P", Set::make(n,n), Domain::unbounded());
            Variable::t c = M->variable("c", n, Domain::unbounded());

            // (1, P(*xi+c)) \in Q 
            for (int i = 0; i < m; ++i)
            {
                auto xi = new_array_ptr<double,1>({ (*x)(i,0), (*x)(i,1),
                                                    (*x)(i,2), (*x)(i,3),
                                                    (*x)(i,4)}); //TODO use for loop

                //auto xi= std::shared_ptr<ndarray<double,1>> (new ndarray<double,1>
                        //(shape_t<1>(n)));
                //for (int j = 0; j < n; ++j) (xi)[j] = (*x)(i, j);

                M->constraint(Expr::vstack(Expr::ones(1), Expr::sub(Expr::mul(P,xi), c)), Domain::inQCone() );
            }

            // t <= det(P)^{1/n}
            //model_utils.det_rootn(M, P, t);
            det_rootn(M,n,P,t);

            // Objective: Maximize t
            M->objective(ObjectiveSense::Maximize, t);
            M->solve();

            return std::make_pair(P->level(),c->level());
        }

    std::ostream & operator<<(std::ostream & os, ndarray<double,1> & a)
    {
        os << "[ ";
        if (a.size() > 0) 
        {
            os << a[0];
            for (int i = 1; i < a.size(); ++i)
                os << "," << a[i];
        }
        os << " ]";
        return os;
    }
} /* LownerJohnEllipsoid */

LJEllipsoidWithPointsAndNormal::LJEllipsoidWithPointsAndNormal(std::vector<Eigen::VectorXd> pointsList_,
                                                               Eigen::VectorXd normalVector_) {
    mPointsList = pointsList_;
    mNormalVector = normalVector_;
    mDim = normalVector_.size();
    mNumPoints = pointsList_.size();
}

LJEllipsoidWithPointsAndNormal::~LJEllipsoidWithPointsAndNormal() {}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> LJEllipsoidWithPointsAndNormal::computeLJEllipsoid() {

    Eigen::MatrixXd indepMat = Eigen::MatrixXd::Identity(mDim, mDim);
    for (int i = 0; i < mDim; ++i) {
        indepMat.block(0, i, mDim, 1) = indepMat.block(0, i, mDim, 1) + mNormalVector;
    }
    indepMat.block(0, 0, mDim, 1) = mNormalVector;
    Eigen::MatrixXd SO = indepMat.householderQr().householderQ();
    mSO = SO;
    // TODO : Should I assert something here?

    auto p= std::shared_ptr<ndarray<double,2>> (new ndarray<double,2>
            (shape_t<2>(mNumPoints,mDim-1)));
    std::vector< Eigen::VectorXd > pointsListLocal(mNumPoints);
    for (int i = 0; i < mNumPoints; ++i) {
        pointsListLocal[i] = SO.transpose() * mPointsList[i];
        for (int j = 0; j < mDim-1; ++j) (*p)(i, j) = (pointsListLocal[i])[j+1];
    }
    double nullDimVal(pointsListLocal[0](0));
    mNullDimVal = nullDimVal;
    auto Pc = LownerJohnEllipsoid::lownerjohn_outer(p);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(mDim-1, mDim-1);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(mDim-1);
    for (int i = 0; i < mDim-1; ++i) {
        for (int j = 0; j < mDim-1; ++j) Q(i, j) = (*Pc.first)(i*(mDim-1)+j);
        c[i] = (*Pc.second)(i);
    }

    Eigen::MatrixXd P = Q.transpose() * Q;
    Eigen::VectorXd b = Q.inverse() * c;

    return std::make_pair(P, b);
}

double LJEllipsoidWithPointsAndNormal::getNullDimVal() {
    return mNullDimVal;
}

Eigen::MatrixXd LJEllipsoidWithPointsAndNormal::getSO() {
    return mSO;
}
