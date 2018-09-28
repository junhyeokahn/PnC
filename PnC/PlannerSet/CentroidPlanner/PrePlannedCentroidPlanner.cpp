#include "PrePlannedCentroidPlanner.hpp"

PrePlannedCentroidPlanner::PrePlannedCentroidPlanner() {

}

PrePlannedCentroidPlanner::~PrePlannedCentroidPlanner() {}

void PrePlannedCentroidPlanner::_doPlan() {
    try {
        Eigen::MatrixXd dummyMat;
        YAML::Node trajectory_cfg =
            YAML::LoadFile(mPreComputedParam->trajectoryFile);
        YAML::Node dynopt_params = trajectory_cfg["dynopt_params"];
        myUtils::readParameter(dynopt_params, "com_motion", dummyMat);
        mRDes.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                mRDes[j][i] = dummyMat(i, j);
            }
        }

        myUtils::readParameter(dynopt_params, "lin_mom", dummyMat);
        mLDes.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                mLDes[j][i] = dummyMat(i, j);
            }
        }

        myUtils::readParameter(dynopt_params, "ang_mom", dummyMat);
        mKDes.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                mKDes[j][i] = dummyMat(i, j);
            }
        }

        myUtils::readParameter(dynopt_params, "time_vec", dummyMat);
        mT.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                mT[j] = dummyMat(i, j);
            }
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
}

void PrePlannedCentroidPlanner::_evalTrajectory( double time,
                                                 Eigen::VectorXd & pos,
                                                 Eigen::VectorXd & vel,
                                                 Eigen::VectorXd & trq) {

}
