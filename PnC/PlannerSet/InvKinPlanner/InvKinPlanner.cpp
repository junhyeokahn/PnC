#include "PnC/PlannerSet/InvKinPlanner/InvKinPlanner.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/ThirdPartyUtilities.hpp"

using namespace myUtils;

void InvKinPlannerParameter::paramSetFromYaml(const std::string & cfg_file) {
    cfgFile = cfg_file;
    saveSolutionFiles = cfg_file.substr(0, cfg_file.size()-12) + "_KINEMATIC_RESULT.yaml";
    try {
        Eigen::MatrixXd dummyMat;
        YAML::Node planner_cfg = YAML::LoadFile(cfgFile.c_str());
        readParameter(planner_cfg, "robot_model_path", robotModelPath);
        robot = std::make_shared<RigidBodyTree<double>>();
        drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM+robotModelPath,
                drake::multibody::joints::kFixed, robot.get());
        initQ = robot->getZeroConfiguration();

        // TODO : What about another robots
        eEfIdx[0] = robot->FindBodyIndex("rAnkle");
        eEfIdx[1] = robot->FindBodyIndex("lAnkle");

        YAML::Node dynopt_params = planner_cfg["dynopt_params"];
        readParameter(dynopt_params, "com_motion", dummyMat);
        rDes.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                rDes[j][i] = dummyMat(i, j);
            }
        }

        readParameter(dynopt_params, "lin_mom", dummyMat);
        lDes.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                lDes[j][i] = dummyMat(i, j);
            }
        }

        readParameter(dynopt_params, "ang_mom", dummyMat);
        kDes.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                kDes[j][i] = dummyMat(i, j);
            }
        }

        readParameter(dynopt_params, "time_vec", dummyMat);
        t.resize(dummyMat.cols());
        for (int i = 0; i < dummyMat.rows(); ++i) {
            for (int j = 0; j < dummyMat.cols(); ++j) {
                t[j] = dummyMat(i, j);
            }
        }
        startTime = t.front();
        endTime = t.back();

        YAML::Node cntopt_params = planner_cfg["cntopt_params"];
        readParameter(cntopt_params, "num_contacts", contactPlanInterface.contactsPerEndeff);
        for (int eef_id=0; eef_id<CentroidModel::numEEf; eef_id++) {
            contactPlanInterface.contactSequence.eEfContacts[eef_id].clear();
            eEfSpline[eef_id].clear();
            eEfPosDes[eef_id].resize(t.size());
            eEfOriDes[eef_id].resize(t.size());
            eEfVelDes[eef_id].resize(t.size());
            if (contactPlanInterface.contactsPerEndeff[eef_id]>0) {
                YAML::Node eff_params = cntopt_params[("eefcnt_" + CentroidModel::eEfIdToString(eef_id)).c_str()];
                for (int cnt_id=0; cnt_id<contactPlanInterface.contactsPerEndeff[eef_id]; cnt_id++) {
                    Eigen::VectorXd v(10);
                    readParameter(eff_params, "cnt"+std::to_string(cnt_id), v);
                    contactPlanInterface.contactSequence.eEfContacts[eef_id].push_back(ContactState());
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeIni = v[0];
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeEnd = v[1];
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].position = v.segment<3>(2);
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].contactType = idToContactType(v(9));
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].orientation = Eigen::Quaternion<double>(v[5],v[6],v[7],v[8]);
                    if (cnt_id > 0) {
                        eEfSpline[eef_id].push_back(BS_Basic<3, 3, 1, 2, 2>());
                        double spline_start_time = contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id-1].timeEnd;
                        double spline_end_time = contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeIni;
                        double start_pva[9]; double end_pva[9];
                        double ** middle_pt = new double*[1];
                        middle_pt[0] = new double[3];
                        for (int i=0; i<3; ++i) {
                            start_pva[i] = contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id-1].position[i];
                            end_pva[i] = contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].position[i];
                        }
                        for (int i=3; i<6; ++i) {
                            start_pva[i] = 0.;
                            end_pva[i] = 0.;
                        }
                        for (int i=0; i<2; ++i) {
                            middle_pt[0][i] = 0.5 * (start_pva[i] + end_pva[i]);
                        }
                        middle_pt[0][2] = end_pva[2] + swingHeight;
                        eEfSpline[eef_id][cnt_id-1].SetParam(start_pva, end_pva,
                                                             middle_pt, spline_end_time - spline_start_time);
                        for (int time_id = 0; time_id < t.size(); ++time_id) {
                            if (t[time_id] >= spline_start_time && t[time_id] <= spline_end_time) {
                                double d_ary[3];
                                eEfSpline[eef_id][cnt_id-1].getCurvePoint(t[time_id] - spline_start_time, d_ary);
                                for (int i = 0; i < 3; ++i) { eEfPosDes[eef_id][time_id][i] = d_ary[i]; }
                                eEfSpline[eef_id][cnt_id-1].getCurveDerPoint(t[time_id] - spline_start_time, 1, d_ary);
                                for (int i = 0; i < 3; ++i) { eEfVelDes[eef_id][time_id][i] = d_ary[i]; }
                                eEfOriDes[eef_id][time_id] = Eigen::Quaternion<double>(1, 0, 0, 0); // TODO : orientation interpolation
                            }
                        }
                        delete[] middle_pt[0];
                        delete[] middle_pt;
                    }
                    for (int time_id = 0; time_id < t.size(); ++time_id) {
                        if (t[time_id] >= contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeIni
                                && t[time_id] <= contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeEnd){
                            eEfPosDes[eef_id][time_id] = contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].position;
                            eEfOriDes[eef_id][time_id] = contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].orientation;
                            eEfVelDes[eef_id][time_id].setZero();
                        }
                    }
                }
            }
        }

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
}

InvKinPlanner::InvKinPlanner() : Planner() {
    mComTol << 0.01, 0.01, 0.03;
    mFootPosTol << 0.0001, 0.0001, 0.0001;
    mFootRPYTol << 0.01, 0.01, 0.01;
    mHTol = Eigen::VectorXd::Zero(6);
    mHTol << 1.0, 1.0, 1.0, 0.05, 0.05, 0.05;
}
InvKinPlanner::~InvKinPlanner() {
}

void InvKinPlanner::_doPlan() {
    mInvKinParam = std::dynamic_pointer_cast<InvKinPlannerParameter>(mParam);
    mQSol.resize(mInvKinParam->t.size(), Eigen::VectorXd::Zero(mInvKinParam->robot->get_num_positions()));
    mQdotSol.resize(mInvKinParam->t.size());

    for (int time_id = 0; time_id < mInvKinParam->t.size(); ++time_id) {
        Eigen::Vector2d tspan(0, 1);
        std::vector<RigidBodyConstraint*> constraint_array;

        Eigen::Vector3d des_com = mInvKinParam->rDes[time_id];

        Eigen::Vector3d foot_offset(0.015, 0., -0.065);
        Eigen::Vector3d des_pos_0 = mInvKinParam->eEfPosDes[0][time_id];
        Eigen::Vector3d des_pos_1 = mInvKinParam->eEfPosDes[1][time_id];

        IKoptions ikoptions(mInvKinParam->robot.get());
        ikoptions.setMajorIterationsLimit(1500);
        ikoptions.setIterationsLimit(20000);
        //ikoptions.setMajorOptimalityTolerance(1E-3);
        //ikoptions.setMajorFeasibilityTolerance(1E-3);
        Eigen::MatrixXd Q =
            Eigen::MatrixXd::Identity(mInvKinParam->robot->get_num_positions(),
                    mInvKinParam->robot->get_num_positions());
        for (int i = 0; i < 3; ++i) Q(i, i) = 0.01;
        for (int i = 3; i < 6; ++i) Q(i, i) = 10.;
        ikoptions.setQ(Q);
        std::vector<std::string> infeasible_constraint;
        int info = 0;

        if (time_id > 0) {
            // 1. r constraint
            WorldCoMConstraint comc(mInvKinParam->robot.get(),
                    des_com - mComTol,
                    des_com + mComTol,
                    tspan);
            constraint_array.push_back(&comc);
            // 2. h constraint
            Eigen::VectorXd des_h(6);
            des_h.head(3) = mInvKinParam->kDes[time_id];
            des_h.tail(3) = mInvKinParam->lDes[time_id];
            KinematicsCache<double> cache = mInvKinParam->robot->doKinematics(mQSol[time_id-1]);
            Eigen::MatrixXd j_cent = mInvKinParam->robot->centroidalMomentumMatrix(cache);
            Eigen::VectorXi rows;
            Eigen::VectorXi cols;
            Eigen::VectorXd vals;
            double dt(mInvKinParam->t[time_id] - mInvKinParam->t[time_id - 1]);
            myUtils::collectNonZeroIdxAndValue(j_cent, rows, cols, vals);
            SingleTimeLinearPostureConstraint hc(mInvKinParam->robot.get(),
                    rows, cols, vals,
                    des_h*dt + j_cent*mQSol[time_id-1] - mHTol*dt,
                    des_h*dt + j_cent*mQSol[time_id-1] + mHTol*dt,
                    tspan);
            //constraint_array.push_back(&hc);
            // 3. ee constraint
            // TODO : consider all 4
            WorldPositionConstraint pc_0(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[0],
                    foot_offset,
                    des_pos_0 - mFootPosTol,
                    des_pos_0 + mFootPosTol,
                    tspan);
            constraint_array.push_back(&pc_0);
            WorldEulerConstraint oc_0(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[0],
                    -mFootRPYTol,
                    mFootRPYTol,
                    tspan);
            constraint_array.push_back(&oc_0);
            WorldPositionConstraint pc_1(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[1],
                    foot_offset,
                    des_pos_1 - mFootPosTol,
                    des_pos_1 + mFootPosTol,
                    tspan);
            constraint_array.push_back(&pc_1);
            WorldEulerConstraint oc_1(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[1],
                    -mFootRPYTol,
                    mFootRPYTol,
                    tspan);
            constraint_array.push_back(&oc_1);
            inverseKin(mInvKinParam->robot.get(), mQSol[time_id-1], mQSol[time_id-1], constraint_array.size(),
                    constraint_array.data(), ikoptions, &(mQSol[time_id]), &info,
                    &infeasible_constraint);
            mQdotSol[time_id] = (mQSol[time_id] - mQSol[time_id-1]) / dt;
        } else {
            // 1. r constraint
            WorldCoMConstraint comc(mInvKinParam->robot.get(),
                    des_com - mComTol,
                    des_com + mComTol,
                    tspan);
            constraint_array.push_back(&comc);
            // 2. ee constraint
            // TODO : consider all 4
            WorldPositionConstraint pc_0(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[0],
                    foot_offset,
                    des_pos_0 - mFootPosTol,
                    des_pos_0 + mFootPosTol,
                    tspan);
            constraint_array.push_back(&pc_0);
            WorldEulerConstraint oc_0(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[0],
                    -mFootRPYTol,
                    mFootRPYTol,
                    tspan);
            constraint_array.push_back(&oc_0);
            WorldPositionConstraint pc_1(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[1],
                    foot_offset,
                    des_pos_1 - mFootPosTol,
                    des_pos_1 + mFootPosTol,
                    tspan);
            constraint_array.push_back(&pc_1);
            WorldEulerConstraint oc_1(mInvKinParam->robot.get(),
                    mInvKinParam->eEfIdx[1],
                    -mFootRPYTol,
                    mFootRPYTol,
                    tspan);
            constraint_array.push_back(&oc_1);
            inverseKin(mInvKinParam->robot.get(), mInvKinParam->initQ, mInvKinParam->initQ, constraint_array.size(),
                    constraint_array.data(), ikoptions, &(mQSol[time_id]), &info,
                    &infeasible_constraint);
            mQdotSol[time_id] = Eigen::VectorXd::Zero(mInvKinParam->robot->get_num_positions());
        }
        std::cout << "At Time : " << mInvKinParam->t[time_id] << "  ";
        if (info == 1) {
            std::cout << "Solution Found" << std::endl;
            //_solutionCheck(time_id);
        } else {
            std::cout << "[info] : " << info << " @ InvKinPlanner.cpp " << std::endl;
            _solutionCheck(time_id);
        }
    }
    _generateCubicTrajectory();
    for (int i = 0; i < mQSol.size(); ++i) {
        myUtils::saveVector(mQSol[i], "ikp_qsol");
        myUtils::saveVector(mQdotSol[i], "ikp_q_dot_sol");
    }
}

void InvKinPlanner::_generateCubicTrajectory() {
    int num_knot(mQSol.size());
    mQSolMat.resize(num_knot);
    mQdotSolMat.resize(num_knot);
    for (int i = 0; i < num_knot; ++i) {
        mQSolMat[i] = mQSol[i];
        mQdotSolMat[i] = mQdotSol[i];
    }

     mQPoly = drake::trajectories::PiecewisePolynomial<double>::Cubic(
             mInvKinParam->t, mQSolMat, mQdotSolMat);
     mQdotPoly = mQPoly.derivative();
     mQddotPoly = mQdotPoly.derivative();
}

void InvKinPlanner::_evalTrajectory( double time,
                                     Eigen::VectorXd & pos,
                                     Eigen::VectorXd & vel,
                                     Eigen::VectorXd & trq ) {
    if (time > mInvKinParam->startTime & time < mInvKinParam->endTime) {
        pos = mQPoly.value(time);
        vel = mQdotPoly.value(time);
        trq = mQddotPoly.value(time);
    }   else {
        std::cout << "Querying wrong time" << std::endl;
    }
}

void InvKinPlanner::_solutionCheck( int time_id ) {

    Eigen::Vector3d foot_offset(0.015, 0., -0.065);
    Eigen::VectorXd q = mQSol[time_id];
    Eigen::VectorXd qdot = mQdotSol[time_id];
    KinematicsCache<double> cache = mInvKinParam->robot->doKinematics(q);
    Eigen::VectorXd act_com = mInvKinParam->robot->centerOfMass(cache);
    Eigen::VectorXd des_com = mInvKinParam->rDes[time_id];
    if (!(myUtils::isInBoundingBox(des_com - 2*mComTol, act_com, des_com + 2*mComTol))) {
        std::cout << "***** CoM *****" << std::endl;
        for (int i = 0; i < 3; ++i)
            std::cout << des_com[i] << "  |  " << act_com[i] << "  |  " << des_com[i] - act_com[i] << std::endl;
    }
    Eigen::VectorXd act_h = mInvKinParam->robot->centroidalMomentumMatrix(cache) * qdot;
    Eigen::VectorXd des_h(6);
    des_h.head(3) = mInvKinParam->kDes[time_id];
    des_h.tail(3) = mInvKinParam->lDes[time_id];
    if (!(myUtils::isInBoundingBox(des_h - 2*mHTol, act_h, des_h + 2*mHTol))) {
        std::cout << "***** h *****" << std::endl;
        for (int i = 0; i < 6; ++i)
            std::cout << des_h[i] << "  |  " << act_h[i] << "  |  " << des_h[i] - act_h[i] << std::endl;
    }
    Eigen::VectorXd act_rf_pos = mInvKinParam->robot->relativeTransform(cache, 0, mInvKinParam->eEfIdx[0]).translation();
    Eigen::VectorXd des_rf_pos = mInvKinParam->eEfPosDes[0][time_id] - foot_offset;
    if (!(myUtils::isInBoundingBox(des_rf_pos - 2*mFootPosTol, act_rf_pos, des_rf_pos + 2*mFootPosTol))) {
        std::cout << "***** Rf pos *****" << std::endl;
        for (int i = 0; i < 3; ++i)
            std::cout << des_rf_pos[i] << "  |  " << act_rf_pos[i] << "  |  " << des_rf_pos[i]- act_rf_pos[i]<< std::endl;
    }
    Eigen::VectorXd act_rf_rpy = mInvKinParam->robot->relativeRollPitchYaw(cache, mInvKinParam->eEfIdx[0], 0);
    if (!(myUtils::isInBoundingBox(-2*mFootRPYTol, act_rf_rpy, 2*mFootRPYTol))) {
        std::cout << "***** Rf rpy *****" << std::endl;
        for (int i = 0; i < 3; ++i)
            std::cout << "0.0" << "  |  " << act_rf_rpy[i] << "  |  " << -act_rf_rpy[i] << std::endl;
    }
    Eigen::VectorXd act_lf_pos = mInvKinParam->robot->relativeTransform(cache, 0, mInvKinParam->eEfIdx[1]).translation();
    Eigen::VectorXd des_lf_pos = mInvKinParam->eEfPosDes[1][time_id] - foot_offset;
    if (!(myUtils::isInBoundingBox(des_lf_pos - 2*mFootPosTol, act_lf_pos, des_lf_pos + 2*mFootPosTol))) {
        std::cout << "***** Lf pos *****" << std::endl;
        for (int i = 0; i < 3; ++i)
            std::cout << des_lf_pos[i] << "  |  " << act_lf_pos[i] << "  |  " << des_lf_pos[i]- act_lf_pos[i]<< std::endl;
    }
    Eigen::VectorXd act_lf_rpy = mInvKinParam->robot->relativeRollPitchYaw(cache, mInvKinParam->eEfIdx[1], 0);
    if (!(myUtils::isInBoundingBox(-2*mFootRPYTol, act_lf_rpy, 2*mFootRPYTol))) {
        std::cout << "***** Lf rpy *****" << std::endl;
        for (int i = 0; i < 3; ++i)
            std::cout << "0.0" << "  |  " << act_lf_rpy[i] << "  |  " << -act_lf_rpy[i] << std::endl;
    }
}
