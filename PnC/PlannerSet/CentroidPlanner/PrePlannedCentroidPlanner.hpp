#pragma once
#include <array>
#include "drake/common/trajectories/piecewise_polynomial.h"

#include "RobotSystem/CentroidModel.hpp"
#include "PnC/Planner.hpp"
#include "Configuration.h"
#include "PnC/PlannerSet/CentroidPlanner/ContactPlanInterface.hpp"
#include "Utils/Math/BSplineBasic.h"

class PrePlannedCentroidPlannerParameter : public PlannerParameter
{
public:
    PrePlannedCentroidPlannerParameter () : PlannerParameter() {};
    virtual ~PrePlannedCentroidPlannerParameter () {};

    std::string trajectoryFile;
    double swingHeight;

private:
    /* data */
};

class PrePlannedCentroidPlanner : public Planner
{
public:
    PrePlannedCentroidPlanner ();
    virtual ~PrePlannedCentroidPlanner ();

    void getContactInfo(const double & time,
                        std::array<bool, CentroidModel::numEEf> & activation_,
                        std::array<double, CentroidModel::numEEf> & phase_,
                        std::array<Eigen::Vector3d, CentroidModel::numEEf> & reaction_forces_);

private:
    virtual void _doPlan();
    /*
     * pos = [dummy, p_com, p_rf, p_lf] \in R^12
     * vel = [k    , l    , v_rf, v_lf] \in R^12
     * acc = [kdot , ldot , a_rf, a_lf] \in R^12
     */
    virtual void _evalTrajectory( double time,
                                  Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel,
                                  Eigen::VectorXd & trq);
    void _generateCubicTrajectory();
    void _debugTrajectory();

    std::shared_ptr<PrePlannedCentroidPlannerParameter> mPreComputedParam;
    double mRobotMass;
    std::vector< Eigen::Vector3d > mRDes;
    Eigen::Vector3d mRIni;
    std::vector< Eigen::Vector3d > mRDotDes;
    std::vector< Eigen::Vector3d > mLDes;
    Eigen::Vector3d mLIni;
    std::vector< Eigen::Vector3d > mLDotDes;
    std::vector< Eigen::Vector3d > mKDes;
    Eigen::Vector3d mKIni;
    std::vector< Eigen::Vector3d > mKDotDes;
    std::vector< double > mT;
    ContactPlanInterface mContactPlanInterface;
    std::array<std::vector<BS_Basic<3, 3, 1, 2, 2>>, CentroidModel::numEEf> mEEfSpline;
    std::array< std::vector<Eigen::Vector3d>, CentroidModel::numEEf > mEEfPosDes;
    std::array< std::vector<Eigen::Quaternion<double>>, CentroidModel::numEEf > mEEfOriDes;
    std::array< std::vector<Eigen::Vector3d>, CentroidModel::numEEf > mEEfVelDes;

    std::vector< Eigen::MatrixXd > mKSolMat;
    std::vector< Eigen::MatrixXd > mKdotSolMat;
    drake::trajectories::PiecewisePolynomial<double> mKPoly;
    drake::trajectories::PiecewisePolynomial<double> mKdotPoly;
    std::vector< Eigen::MatrixXd > mRSolMat;
    std::vector< Eigen::MatrixXd > mRdotSolMat;
    drake::trajectories::PiecewisePolynomial<double> mRPoly;
    drake::trajectories::PiecewisePolynomial<double> mRdotPoly;
    drake::trajectories::PiecewisePolynomial<double> mRddotPoly;
    std::vector< Eigen::MatrixXd > mRfSolMat;
    std::vector< Eigen::MatrixXd > mRfdotSolMat;
    drake::trajectories::PiecewisePolynomial<double> mRfPoly;
    drake::trajectories::PiecewisePolynomial<double> mRfdotPoly;
    drake::trajectories::PiecewisePolynomial<double> mRfddotPoly;
    std::vector< Eigen::MatrixXd > mLfSolMat;
    std::vector< Eigen::MatrixXd > mLfdotSolMat;
    drake::trajectories::PiecewisePolynomial<double> mLfPoly;
    drake::trajectories::PiecewisePolynomial<double> mLfdotPoly;
    drake::trajectories::PiecewisePolynomial<double> mLfddotPoly;

};
