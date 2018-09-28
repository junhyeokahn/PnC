#include "PrePlannedCentroidPlanner.hpp"

PrePlannedCentroidPlanner::PrePlannedCentroidPlanner() {

    printf("[Pre-Planned Centroid Planner] Construted\n");
}

PrePlannedCentroidPlanner::~PrePlannedCentroidPlanner() {}

void PrePlannedCentroidPlanner::_doPlan() {
    mPreComputedParam = std::dynamic_pointer_cast<PrePlannedCentroidPlannerParameter>(mParam);
    try {
        Eigen::MatrixXd dummyMat;
        YAML::Node trajectory_cfg =
            YAML::LoadFile(mPreComputedParam->trajectoryFile);
        YAML::Node dynopt_params = trajectory_cfg["dynopt_params"];

        myUtils::readParameter(dynopt_params, "robot_mass", mRobotMass);

        myUtils::readParameter(dynopt_params, "time_vec", dummyMat);
        mT.resize(dummyMat.cols()+1);
        mT[0] = 0.0;
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 1; j < dummyMat.cols(); ++j) {
                mT[j] = dummyMat(i, j);
            }
        }

        myUtils::readParameter(dynopt_params, "com_motion", dummyMat);
        myUtils::readParameter(dynopt_params, "ini_com", mRIni);
        mRDes.resize(dummyMat.cols()+1);
        mRDotDes.resize(dummyMat.cols()+1);
        mRDes[0] = mRIni;
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 1; j < dummyMat.cols(); ++j) {
                mRDes[j][i] = dummyMat(i, j);
            }
        }
        for (int i = 0; i < mRDes.size(); ++i) {
                if (i == 0) { mRDotDes[i].setZero(); }
                else { mRDotDes[i] = ( mRDes[i] - mRDes[i-1] ) / ( mT[i] - mT[i-1] ); }
        }

        myUtils::readParameter(dynopt_params, "lin_mom", dummyMat);
        myUtils::readParameter(dynopt_params, "ini_lin_mom", mLIni);
        mLDes.resize(dummyMat.cols()+1);
        mLDotDes.resize(dummyMat.cols()+1);
        mLDes[0] = mLIni;
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                mLDes[j][i] = dummyMat(i, j);
            }
        }
        for (int i = 0; i < mLDotDes.size(); ++i) {
            if (i == 0) { mLDotDes[i].setZero(); }
            else { mLDotDes[i] = ( mLDes[i] - mLDes[i-1] ) / ( mT[i] - mT[i-1] ); }
        }


        myUtils::readParameter(dynopt_params, "ang_mom", dummyMat);
        myUtils::readParameter(dynopt_params, "ini_ang_mom", mKIni);
        mKDes.resize(dummyMat.cols()+1);
        mKDotDes.resize(dummyMat.cols()+1);
        mKDes[0] = mKIni;
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                mKDes[j][i] = dummyMat(i, j);
            }
        }
        for (int i = 0; i < mKDes.size(); ++i) {
            if (i == 0) { mKDotDes[i].setZero(); }
            else { mKDotDes[i] = ( mKDes[i] - mKDes[i-1] ) / ( mT[i] - mT[i-1] ); }
        }

        mPreComputedParam->startTime = mT.front();
        mPreComputedParam->endTime = mT.back();

        YAML::Node cntopt_params = trajectory_cfg["cntopt_params"];
        myUtils::readParameter(cntopt_params, "num_contacts", mContactPlanInterface.contactsPerEndeff);
        for (int eef_id=0; eef_id<CentroidModel::numEEf; eef_id++) {
            mContactPlanInterface.contactSequence.eEfContacts[eef_id].clear();
            mEEfSpline[eef_id].clear();
            mEEfPosDes[eef_id].resize(mT.size());
            mEEfOriDes[eef_id].resize(mT.size());
            mEEfVelDes[eef_id].resize(mT.size());
            if (mContactPlanInterface.contactsPerEndeff[eef_id]>0) {
                YAML::Node eff_params = cntopt_params[("eefcnt_" + CentroidModel::eEfIdToString(eef_id)).c_str()];
                for (int cnt_id=0; cnt_id<mContactPlanInterface.contactsPerEndeff[eef_id]; cnt_id++) {
                    Eigen::VectorXd v(10);
                    myUtils::readParameter(eff_params, "cnt"+std::to_string(cnt_id), v);
                    mContactPlanInterface.contactSequence.eEfContacts[eef_id].push_back(ContactState());
                    mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeIni = v[0];
                    mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeEnd = v[1];
                    mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].position = v.segment<3>(2);
                    mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].contactType = idToContactType(v(9));
                    mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].orientation = Eigen::Quaternion<double>(v[5],v[6],v[7],v[8]);
                    if (cnt_id > 0) {
                        mEEfSpline[eef_id].push_back(BS_Basic<3, 3, 1, 2, 2>());
                        double spline_start_time = mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id-1].timeEnd;
                        double spline_end_time = mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeIni;
                        double start_pva[9]; double end_pva[9];
                        double ** middle_pt = new double*[1];
                        middle_pt[0] = new double[3];
                        for (int i=0; i<3; ++i) {
                            start_pva[i] = mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id-1].position[i];
                            end_pva[i] = mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].position[i];
                        }
                        for (int i=3; i<6; ++i) {
                            start_pva[i] = 0.;
                            end_pva[i] = 0.;
                        }
                        for (int i=0; i<2; ++i) {
                            middle_pt[0][i] = 0.5 * (start_pva[i] + end_pva[i]);
                        }
                        middle_pt[0][2] = end_pva[2] + mPreComputedParam->swingHeight;
                        mEEfSpline[eef_id][cnt_id-1].SetParam(start_pva, end_pva,
                                                             middle_pt, spline_end_time - spline_start_time);
                        for (int time_id = 0; time_id < mT.size(); ++time_id) {
                            if (mT[time_id] >= spline_start_time && mT[time_id] <= spline_end_time) {
                                double d_ary[3];
                                mEEfSpline[eef_id][cnt_id-1].getCurvePoint(mT[time_id] - spline_start_time, d_ary);
                                for (int i = 0; i < 3; ++i) { mEEfPosDes[eef_id][time_id][i] = d_ary[i]; }
                                mEEfSpline[eef_id][cnt_id-1].getCurveDerPoint(mT[time_id] - spline_start_time, 1, d_ary);
                                for (int i = 0; i < 3; ++i) { mEEfVelDes[eef_id][time_id][i] = d_ary[i]; }
                                mEEfOriDes[eef_id][time_id] = Eigen::Quaternion<double>(1, 0, 0, 0); // TODO : orientation interpolation
                            }
                        }
                        delete[] middle_pt[0];
                        delete[] middle_pt;
                    }
                    for (int time_id = 0; time_id < mT.size(); ++time_id) {
                        if (mT[time_id] >= mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeIni
                                && mT[time_id] <= mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeEnd){
                            mEEfPosDes[eef_id][time_id] = mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].position;
                            mEEfOriDes[eef_id][time_id] = mContactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].orientation;
                            mEEfVelDes[eef_id][time_id].setZero();
                        }
                    }
                }
            }
        }
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
    _debugTrajectory();
    _generateCubicTrajectory();
}

void PrePlannedCentroidPlanner::_generateCubicTrajectory() {
    int num_knot(mRDes.size());
    mKSolMat.resize(num_knot); mKdotSolMat.resize(num_knot);
    mRSolMat.resize(num_knot); mRdotSolMat.resize(num_knot);
    mRfSolMat.resize(num_knot); mRfdotSolMat.resize(num_knot);
    mLfSolMat.resize(num_knot); mLfdotSolMat.resize(num_knot);
    for (int i = 0; i < num_knot; ++i) {
        mKSolMat[i] = mKDes[i]; mKdotSolMat[i] = mKdotSolMat[i];
        mRSolMat[i] = mRDes[i]; mRdotSolMat[i] = mRDotDes[i];
        mRfSolMat[i] = mEEfPosDes[0][i]; mRfdotSolMat[i] = mEEfVelDes[0][i];
        mLfSolMat[i] = mEEfPosDes[1][i]; mLfdotSolMat[i] = mEEfVelDes[1][i];
    }
    mKPoly = drake::trajectories::PiecewisePolynomial<double>::Cubic(
            mT, mKSolMat, mKdotSolMat);
    mKdotPoly = mKPoly.derivative();
    mRPoly = drake::trajectories::PiecewisePolynomial<double>::Cubic(
            mT, mRSolMat, mRdotSolMat);
    mRdotPoly = mRPoly.derivative();
    mRddotPoly = mRdotPoly.derivative();
    mRfPoly = drake::trajectories::PiecewisePolynomial<double>::Cubic(
            mT, mRfSolMat, mRfdotSolMat);
    mRfdotPoly = mRfPoly.derivative();
    mRfddotPoly = mRfdotPoly.derivative();
    mLfPoly = drake::trajectories::PiecewisePolynomial<double>::Cubic(
            mT, mLfSolMat, mLfdotSolMat);
    mLfdotPoly = mLfPoly.derivative();
    mLfddotPoly = mLfdotPoly.derivative();
}

void PrePlannedCentroidPlanner::_evalTrajectory( double time,
                                                 Eigen::VectorXd & pos,
                                                 Eigen::VectorXd & vel,
                                                 Eigen::VectorXd & trq) {
    pos = Eigen::VectorXd::Zero(12);
    vel = Eigen::VectorXd::Zero(12);
    trq = Eigen::VectorXd::Zero(12);
    if (time > mPreComputedParam->startTime & time < mPreComputedParam->endTime) {
        pos.segment(3, 3) = mRPoly.value(time);
        pos.segment(6, 3) = mRfPoly.value(time);
        pos.segment(9, 3) = mLfPoly.value(time);
        vel.segment(0, 3) = mKPoly.value(time);
        vel.segment(3, 3) = mRobotMass * mRdotPoly.value(time);
        vel.segment(6, 3) = mRfdotPoly.value(time);
        vel.segment(9, 3) = mLfdotPoly.value(time);
        trq.segment(0, 3) = mKdotPoly.value(time);
        trq.segment(3, 3) = mRobotMass * mRddotPoly.value(time);
        trq.segment(6, 3) = mRfddotPoly.value(time);
        trq.segment(9, 3) = mLfddotPoly.value(time);
    }
}

void PrePlannedCentroidPlanner::_debugTrajectory(){
    int num_knot(mRDes.size());
    for (int i = 0; i < num_knot; ++i) {
        myUtils::saveVector(mRDes[i], "r_des");
        myUtils::saveVector(mRDotDes[i], "rdot_des");
        myUtils::saveVector(mLDes[i], "l_des");
        myUtils::saveVector(mLDotDes[i], "ldot_des");
        myUtils::saveVector(mKDes[i], "k_des");
        myUtils::saveVector(mKDotDes[i], "kdot_des");
        myUtils::saveValue(mT[i], "t");
        myUtils::saveVector(( Eigen::VectorXd )mEEfPosDes[0][i], "rf_des");
        myUtils::saveVector(( Eigen::VectorXd )mEEfPosDes[1][i], "lf_des");
        myUtils::saveVector(( Eigen::VectorXd )mEEfVelDes[0][i], "rfdot_des");
        myUtils::saveVector(( Eigen::VectorXd )mEEfVelDes[1][i], "lfdot_des");
    }
    std::cout << "let's double check traj" << std::endl; //TODO
    exit(0);
}
