#include "PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp"
#include "Utils/IO/IOUtilities.hpp"
#include "Utils/Math/minjerk_one_dim.hpp"

using namespace myUtils;
using namespace solver;

CentroidPlannerParameter::CentroidPlannerParameter(YAML::Node planner_cfg,
                                                   double robot_mass) {
    myUtils::pretty_constructor(2, "Centroidal Planner");

    b_req.resize(4, false);

    defaultSolverSettingFile =
        THIS_COM +
        std::string("Config/Solver/DEFAULT_CONIC_SOLVER_SETTING.yaml");

    try {
        // =====================================================================
        // Solver Setting
        // =====================================================================
        YAML::Node optim_cfg = planner_cfg["optimization"];
        saveDynamicsFile =
            THIS_COM + readParameter<std::string>(optim_cfg, "storage_path");
        YAML::Node contact_cfg = planner_cfg["contact_sequence"];
        // Dynamics parameters
        readParameter(optim_cfg, "num_com_viapoints", numComViaPoints);
        comViaPoints.clear();
        for (int via_id = 0; via_id < numComViaPoints; via_id++) {
            comViaPoints.push_back(Eigen::Vector4d::Zero());
            readParameter(optim_cfg["com_viapoints"],
                          "via" + std::to_string(via_id), comViaPoints[via_id]);
        }

        readParameter(optim_cfg, "time_step", timeStep);
        readParameter(optim_cfg, "n_act_eefs", numActEEfs);
        readParameter(optim_cfg, "time_horizon", timeHorizon);
        readParameter(optim_cfg, "external_force", externalForce);
        if (readParameter<std::string>(optim_cfg, "heuristic")
                .compare("TrustRegion") == 0) {
            heuristic = Heuristic::trustRegion;
        } else if (readParameter<std::string>(optim_cfg, "heuristic")
                       .compare("SoftConstraint") == 0) {
            heuristic = Heuristic::softConstraint;
        } else if (readParameter<std::string>(optim_cfg, "heuristic")
                       .compare("TimeOptimization") == 0) {
            heuristic = Heuristic::timeOptimization;
        } else {
            heuristic = Heuristic::softConstraint;
        }
        // Configuration parameters
        gravity = 9.81;
        robotMass = robot_mass;
        readParameter(optim_cfg, "torque_range", torqueRange);
        readParameter(optim_cfg, "friction_coeff", frictionCoeff);
        readParameter(optim_cfg, "max_eef_lengths", maxEEfLengths);
        // readParameter(optim_cfg, "min_eef_lengths", minEEfLengths);
        for (int eef_id = 0; eef_id < CentroidModel::numEEf; eef_id++) {
            readParameter(optim_cfg,
                          "cop_range_" + CentroidModel::eEfIdToString(eef_id),
                          copRange[eef_id]);
            readParameter(optim_cfg,
                          "eef_offset_" + CentroidModel::eEfIdToString(eef_id),
                          eEfOffset[eef_id]);
        }
        isFrictionConeLinear =
            (readParameter<std::string>(optim_cfg, "friction_cone")
                 .compare("LinearCone") == 0);
        // Dynamics weights
        readParameter(optim_cfg, "w_com", wCom);
        readParameter(optim_cfg, "w_amom", wAMom);
        readParameter(optim_cfg, "w_lmom", wLMom);
        readParameter(optim_cfg, "w_amomd", wAMomD);
        readParameter(optim_cfg, "w_lmomd", wLMomD);
        readParameter(optim_cfg, "w_com_via", wComVia);
        readParameter(optim_cfg, "w_trq_arm", wTrqArm);
        readParameter(optim_cfg, "w_trq_leg", wTrqLeg);
        readParameter(optim_cfg, "w_frc_arm", wFrcArm);
        readParameter(optim_cfg, "w_frc_leg", wFrcLeg);
        readParameter(optim_cfg, "w_dfrc_arm", wDFrcArm);
        readParameter(optim_cfg, "w_dfrc_leg", wDFrcLeg);
        readParameter(optim_cfg, "w_amom_final", wAMomFinal);
        readParameter(optim_cfg, "w_lmom_final", wLMomFinal);
        readParameter(optim_cfg, "w_amom_track", wAMomTrack);
        readParameter(optim_cfg, "w_lmom_track", wLMomTrack);
        // Storage information
        readParameter(optim_cfg, "store_data", isStoreData);
        // Solver setting
        readParameter(optim_cfg, "use_default_solver_setting",
                      isDefaultSolverSetting);
        massTimesGravity = robotMass * gravity;
        gravityVector = Eigen::Vector3d(0., 0., -gravity);
        numTimeSteps = std::floor(timeHorizon / timeStep);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

void CentroidPlannerParameter::UpdateTimeHorizon(double time_horizon) {
    timeHorizon = time_horizon;
    numTimeSteps = std::floor(timeHorizon / timeStep);
}

void CentroidPlannerParameter::UpdateContactPlanInterface(
    std::array<std::vector<Eigen::VectorXd>, CentroidModel::numEEf>
        contact_sequence) {
    Eigen::VectorXd _num_contacts = Eigen::VectorXd::Zero(4);
    YAML::Node _eefcnt;
    // fill out contactsequence
    for (int eef_id = 0; eef_id < CentroidModel::numEEf; ++eef_id) {
        int cpe = contact_sequence[eef_id].size();
        _num_contacts[eef_id] = cpe;
        // std::cout << "eef_id : " << eef_id << std::endl;
        contactPlanInterface.contactsPerEndeff[eef_id] = cpe;
        contactPlanInterface.contactSequence.eEfContacts[eef_id].clear();
        for (int c_id = 0; c_id < cpe; ++c_id) {
            Eigen::VectorXd v = contact_sequence[eef_id][c_id];
            contactPlanInterface.contactSequence.eEfContacts[eef_id].push_back(
                ContactState());
            contactPlanInterface.contactSequence.eEfContacts[eef_id][c_id]
                .timeIni = v[0];
            contactPlanInterface.contactSequence.eEfContacts[eef_id][c_id]
                .timeEnd = v[1];
            contactPlanInterface.contactSequence.eEfContacts[eef_id][c_id]
                .position = v.segment<3>(2);
            contactPlanInterface.contactSequence.eEfContacts[eef_id][c_id]
                .orientation =
                Eigen::Quaternion<double>(v[5], v[6], v[7], v[8]);
            contactPlanInterface.contactSequence.eEfContacts[eef_id][c_id]
                .contactType = idToContactType(static_cast<int>(v(9)));
            _eefcnt["eefcnt_" + CentroidModel::eEfIdToString(eef_id)]
                   ["cnt" + std::to_string(c_id)] = v;
            // std::cout << "c_id : " << c_id << std::endl;
            // std::cout << contactPlanInterface.contactSequence
            //.eEfContacts[eef_id][c_id]
            //<< std::endl;
        }
    }
    _eefcnt["num_contacts"] = _num_contacts;
    contact_plan = _eefcnt;
    b_req[2] = true;
}

void CentroidPlannerParameter::UpdateInitialState(
    const Eigen::Vector3d& r, const Eigen::Vector3d& l,
    const Eigen::Vector3d& k,
    const std::array<int, CentroidModel::numEEf>& activation,
    const std::array<Eigen::Vector3d, CentroidModel::numEEf>& eef_frc,
    const std::array<Eigen::Isometry3d, CentroidModel::numEEf>& iso) {
    initialState.com = r;
    initialState.lMom = l;
    initialState.aMom = k;
    for (int eef_id = 0; eef_id < CentroidModel::numEEf; ++eef_id) {
        initialState.eEfsActivation[eef_id] = activation[eef_id];
        initialState.eEfsFrc[eef_id] = eef_frc[eef_id];
        initialState.eEfsPosition[eef_id] = iso[eef_id].translation();
        initialState.eEfsOrientation[eef_id] =
            Eigen::Quaternion<double>(iso[eef_id].linear());
    }
    b_req[0] = true;
    // std::cout << "initial state" << std::endl;
    // std::cout << initialState << std::endl;
}

void CentroidPlannerParameter::UpdateTerminalState(
    const Eigen::Vector3d& r_displacement) {
    comDisplacement = r_displacement;
    b_req[3] = true;
    // std::cout << "com goal" << std::endl;
    // std::cout << initialState.com + comDisplacement << std::endl;
}

void CentroidPlannerParameter::UpdateRefDynamicsStateSequence() {
    refDynamicsStateSequence.resize(numTimeSteps);
    b_req[1] = true;
}

CentroidPlanner::CentroidPlanner(CentroidPlannerParameter* _param) : Planner() {
    mCentParam = _param;
}

CentroidPlanner::~CentroidPlanner() {}

void CentroidPlanner::DoPlan(bool b_use_previous_solution) {
    for (int i = 0; i < mCentParam->b_req.size(); ++i) {
        assert(mCentParam->b_req[i]);
        mCentParam->b_req[i] = false;
    }
    _initialize(b_use_previous_solution);
    _optimize();
}

void CentroidPlanner::EvalTrajectory(double time, Eigen::VectorXd& s,
                                     Eigen::VectorXd& sdot,
                                     Eigen::VectorXd& u) {
    s = Eigen::VectorXd::Zero(6);
    sdot = Eigen::VectorXd::Zero(6);
    // TODO : u?

    double ini_time(0.);
    Eigen::Vector3d ini_com, ini_amom, ini_lmom;

    double fin_time(0.);
    Eigen::Vector3d fin_com, fin_amom, fin_lmom;

    int idx(-1);

    for (int i = 0; i < mCentParam->numTimeSteps; ++i) {
        ini_time = mCentParam->timeStep * (i);
        fin_time = mCentParam->timeStep * (i + 1);
        if (ini_time <= time && time <= fin_time) {
            idx = i;
            break;
        }
    }

    if (idx == -1) {
        // Time Horizon is Finished. Interpolate to the goal
        static double ini_time(time);
        double interp_dur(4.);
        ini_com =
            mDynStateSeq.dynamicsStateSequence[mCentParam->numTimeSteps - 1]
                .com;
        ini_amom =
            mDynStateSeq.dynamicsStateSequence[mCentParam->numTimeSteps - 1]
                .aMom;
        ini_lmom =
            mDynStateSeq.dynamicsStateSequence[mCentParam->numTimeSteps - 1]
                .lMom;
        if (time >= ini_time + interp_dur) {
            for (int i = 0; i < 3; ++i) {
                s[i + 3] = mComPosGoal[i];
                sdot[i] = 0.;
                sdot[i + 3] = 0.;
            }
        } else {
            for (int i = 0; i < 3; ++i) {
                s[i + 3] = ((mComPosGoal[i] - ini_com[i]) / (interp_dur) *
                            (time - ini_time)) +
                           ini_com[i];

                sdot[i] =
                    ((0. - ini_amom[i]) / (interp_dur) * (time - ini_time)) +
                    ini_amom[i];
                sdot[i + 3] =
                    ((0. - ini_lmom[i]) / (interp_dur) * (time - ini_time)) +
                    ini_lmom[i];
            }
        }
    } else {
        // Time Horizon is not Finished. Return the planned trajectory
        if (idx == 0) {
            ini_com = mCentParam->initialState.com;
            ini_amom = mCentParam->initialState.aMom;
            ini_lmom = mCentParam->initialState.lMom;
        } else {
            ini_com = mDynStateSeq.dynamicsStateSequence[idx - 1].com;
            ini_amom = mDynStateSeq.dynamicsStateSequence[idx - 1].aMom;
            ini_lmom = mDynStateSeq.dynamicsStateSequence[idx - 1].lMom;
        }
        fin_com = mDynStateSeq.dynamicsStateSequence[idx].com;
        fin_amom = mDynStateSeq.dynamicsStateSequence[idx].aMom;
        fin_lmom = mDynStateSeq.dynamicsStateSequence[idx].lMom;

        // =========================================================================
        // cubic interpolation
        // =========================================================================
        // double ini_time_3 = std::pow(ini_time, 3);
        // double ini_time_2 = std::pow(ini_time, 2);
        // double fin_time_3 = std::pow(fin_time, 3);
        // double fin_time_2 = std::pow(fin_time, 2);
        // double t3 = std::pow(time, 3);
        // double t2 = std::pow(time, 2);
        // for (int i = 0; i < 3; ++i) {
        // Eigen::Matrix4d A;
        // Eigen::Vector4d b;
        // Eigen::Vector4d coef;
        // A << ini_time_3, ini_time_2, ini_time, 1, 3 * ini_time_2, 2 *
        // ini_time, 1, 0, fin_time_3, fin_time_2, fin_time, 1, 3 * fin_time_2,
        // 2 * fin_time, 1, 0;
        // b << ini_com[i], ini_lmom[i] / mCentParam->robotMass, fin_com[i],
        // fin_lmom[i] / mCentParam->robotMass;
        // coef = A.colPivHouseholderQr().solve(b);
        // s[i + 3] = coef[0] * t3 + coef[1] * t2 + coef[2] * time + coef[3];
        // sdot[i + 3] = (3 * coef[0] * t2 + 2 * coef[1] * time + coef[2]) *
        // mCentParam->robotMass;
        // sdot[i] = ((fin_amom[i] - ini_amom[i]) / (fin_time - ini_time) *
        //(time - ini_time)) +
        // ini_amom[i];
        //}

        // =========================================================================
        // min jerk interpolation
        // =========================================================================
        /*    for (int i = 0; i < 3; ++i) {*/
        // Eigen::Vector3d ini, fin;
        // ini << ini_com[i], ini_lmom[i] / mCentParam->robotMass, 0;
        // fin << fin_com[i], fin_lmom[i] / mCentParam->robotMass, 0;
        // MinJerk_OneDimension min_jerk =
        // MinJerk_OneDimension(ini, fin, ini_time, fin_time);
        // double tmp(0);
        // min_jerk.getPos(time, tmp);
        // s[i + 3] = tmp;
        // min_jerk.getVel(time, tmp);
        // sdot[i + 3] = tmp * mCentParam->robotMass;
        // sdot[i] = ((fin_amom[i] - ini_amom[i]) / (fin_time - ini_time) *
        //(time - ini_time)) +
        // ini_amom[i];
        /*}*/

        // linear interpolation
        for (int i = 0; i < 3; ++i) {
            s[i + 3] = ((fin_com[i] - ini_com[i]) / (fin_time - ini_time) *
                        (time - ini_time)) +
                       ini_com[i];
            sdot[i + 3] = ((fin_lmom[i] - ini_lmom[i]) / (fin_time - ini_time) *
                           (time - ini_time)) +
                          ini_lmom[i];
            sdot[i] = ((fin_amom[i] - ini_amom[i]) / (fin_time - ini_time) *
                       (time - ini_time)) +
                      ini_amom[i];
        }
    }
}

void CentroidPlanner::_initialize(bool b_use_previous_solution) {
    if (!mCentParam->isDefaultSolverSetting) {
        mModel.configSetting(mCentParam->cfgFile);
    } else {
        mModel.configSetting(mCentParam->defaultSolverSettingFile);
    }

    mComPosGoal = mCentParam->initialState.com + mCentParam->comDisplacement;
    mFrictionCone.getCone(mCentParam->frictionCoeff, mConeMatrix);
    mDynStateSeq.resize(mCentParam->numTimeSteps);
    for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].time = mCentParam->timeStep;

    mCentParam->contactPlanInterface.fillDynamicsSequence(
        mDynStateSeq, mCentParam->numTimeSteps, mCentParam->timeStep);

    if (!b_use_previous_solution) {
        _initializeOptimizationVariables();
    }
}

void CentroidPlanner::_initializeOptimizationVariables() {
    mNumVars = 0.;
    double inf_value = SolverSetting::inf;

    // center of mass, linear and angular momentum
    mCom.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value,
                    mNumVars);
    mLMom.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value,
                     mNumVars);
    mAMom.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value,
                     mNumVars);

    // upper and lower bound variables, forces, cops, torques
    for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
        mLbVar[eef_id].initialize('C', 3, mDynStateSeq.activeEEfSteps[eef_id],
                                  -inf_value, inf_value, mNumVars);
        mUbVar[eef_id].initialize('C', 3, mDynStateSeq.activeEEfSteps[eef_id],
                                  -inf_value, inf_value, mNumVars);
        mFrcWorld[eef_id].initialize('C', 3,
                                     mDynStateSeq.activeEEfSteps[eef_id],
                                     -inf_value, inf_value, mNumVars);
        mCopLocal[eef_id].initialize('C', 2,
                                     mDynStateSeq.activeEEfSteps[eef_id],
                                     -inf_value, inf_value, mNumVars);
        mTrqLocal[eef_id].initialize('C', 1,
                                     mDynStateSeq.activeEEfSteps[eef_id],
                                     -inf_value, inf_value, mNumVars);
    }
}

solver::ExitCode CentroidPlanner::_optimize(bool update_tracking_objective) {
    if (update_tracking_objective) _updateTrackingObjective();

    mSolveTime = 0.0;
    mHasConverged = false;
    _internalOptimize(true);

    _saveToFile(mCentParam->refDynamicsStateSequence);
    return mExitCode;
}

void CentroidPlanner::_updateTrackingObjective() {
    mCentParam->wLMom = mCentParam->wLMomTrack;
    mCentParam->wAMom = mCentParam->wAMomTrack;
}

void CentroidPlanner::_internalOptimize(bool is_first_time) {
    try {
        // =====================================================================
        // add variables to model
        // =====================================================================
        mVars.clear();
        for (int var_id = 0; var_id < mNumVars; var_id++)
            mVars.push_back(Var());

        mModel.clean();
        _addVariableToModel(mCom, mModel, mVars);
        _addVariableToModel(mLMom, mModel, mVars);
        _addVariableToModel(mAMom, mModel, mVars);

        for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
            _addVariableToModel(mLbVar[eef_id], mModel, mVars);
            _addVariableToModel(mUbVar[eef_id], mModel, mVars);
            _addVariableToModel(mFrcWorld[eef_id], mModel, mVars);
            _addVariableToModel(mCopLocal[eef_id], mModel, mVars);
            _addVariableToModel(mTrqLocal[eef_id], mModel, mVars);
        }

        // =====================================================================
        // adding quadratic objective
        // =====================================================================
        mQuadObjective.clear();

        if (mCentParam->numComViaPoints > 0) {
            for (int via_id = 0; via_id < mCentParam->numComViaPoints; via_id++)
                for (int axis_id = 0; axis_id < 3; axis_id++)
                    mQuadObjective.addQuaTerm(
                        mCentParam->wComVia[axis_id],
                        LinExpr(mVars[mCom.id(
                            axis_id, int(mCentParam->comViaPoints[via_id](0) /
                                         mCentParam->timeStep))]) -
                            LinExpr(
                                mCentParam->comViaPoints[via_id](axis_id + 1)));
        }

        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
            for (int axis_id = 0; axis_id < 3; axis_id++) {
                // penalty on center of mass, linear and angular momentum
                if (time_id == mCentParam->numTimeSteps - 1) {
                    mQuadObjective.addQuaTerm(
                        mCentParam->wCom[axis_id],
                        LinExpr(mVars[mCom.id(axis_id, time_id)]) -
                            LinExpr(mComPosGoal[axis_id]));
                    mQuadObjective.addQuaTerm(
                        mCentParam->wLMomFinal[axis_id],
                        LinExpr(mVars[mLMom.id(axis_id, time_id)]) -
                            LinExpr(mCentParam->refDynamicsStateSequence
                                        .dynamicsStateSequence[time_id]
                                        .lMom[axis_id]));
                    mQuadObjective.addQuaTerm(
                        mCentParam->wAMomFinal[axis_id],
                        LinExpr(mVars[mAMom.id(axis_id, time_id)]) -
                            LinExpr(mCentParam->refDynamicsStateSequence
                                        .dynamicsStateSequence[time_id]
                                        .aMom[axis_id]));
                } else {
                    mQuadObjective.addQuaTerm(
                        mCentParam->wLMom[axis_id],
                        LinExpr(mVars[mLMom.id(axis_id, time_id)]) -
                            LinExpr(mCentParam->refDynamicsStateSequence
                                        .dynamicsStateSequence[time_id]
                                        .lMom[axis_id]));
                    mQuadObjective.addQuaTerm(
                        mCentParam->wAMom[axis_id],
                        LinExpr(mVars[mAMom.id(axis_id, time_id)]) -
                            LinExpr(mCentParam->refDynamicsStateSequence
                                        .dynamicsStateSequence[time_id]
                                        .aMom[axis_id]));
                }

                // penalty on linear and angular momentum rates
                if (time_id == 0) {
                    mQuadObjective.addQuaTerm(
                        mCentParam->wLMomD[axis_id],
                        (LinExpr(mVars[mLMom.id(axis_id, time_id)]) -
                         LinExpr(mCentParam->initialState.lMom[axis_id])) *
                            (1.0 /
                             mDynStateSeq.dynamicsStateSequence[time_id].time));
                    mQuadObjective.addQuaTerm(
                        mCentParam->wAMomD[axis_id],
                        (LinExpr(mVars[mAMom.id(axis_id, time_id)]) -
                         LinExpr(mCentParam->initialState.aMom[axis_id])) *
                            (1.0 /
                             mDynStateSeq.dynamicsStateSequence[time_id].time));
                } else {
                    // TEST
                    if (axis_id == 2) {
                        mQuadObjective.addQuaTerm(
                           mCentParam->wCom[axis_id]/10.0,
                           LinExpr(mVars[mCom.id(axis_id, time_id)]) -
                               LinExpr(mComPosGoal[axis_id]));
                    }
                    // TEST
                    mQuadObjective.addQuaTerm(
                        mCentParam->wLMomD[axis_id],
                        (LinExpr(mVars[mLMom.id(axis_id, time_id)]) -
                         LinExpr(mVars[mLMom.id(axis_id, time_id - 1)])) *
                            (1.0 /
                             mDynStateSeq.dynamicsStateSequence[time_id].time));
                    mQuadObjective.addQuaTerm(
                        mCentParam->wAMomD[axis_id],
                        (LinExpr(mVars[mAMom.id(axis_id, time_id)]) -
                         LinExpr(mVars[mAMom.id(axis_id, time_id - 1)])) *
                            (1.0 /
                             mDynStateSeq.dynamicsStateSequence[time_id].time));
                }

                // penalty on forces
                for (int eef_id = 0; eef_id < mCentParam->numActEEfs;
                     eef_id++) {
                    if (mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id]) {
                        if (eef_id == static_cast<int>(
                                          CentroidModel::EEfID::rightFoot) ||
                            eef_id == static_cast<int>(
                                          CentroidModel::EEfID::leftFoot)) {
                            mQuadObjective.addQuaTerm(
                                mCentParam->wFrcLeg[axis_id],
                                mVars[mFrcWorld[eef_id].id(
                                    axis_id,
                                    mDynStateSeq.dynamicsStateSequence[time_id]
                                        .eEfsActivationIds[eef_id])]);
                        } else {
                            mQuadObjective.addQuaTerm(
                                mCentParam->wFrcArm[axis_id],
                                mVars[mFrcWorld[eef_id].id(
                                    axis_id,
                                    mDynStateSeq.dynamicsStateSequence[time_id]
                                        .eEfsActivationIds[eef_id])]);
                        }
                    }
                }

                // penalty on rate of forces
                for (int eef_id = 0; eef_id < mCentParam->numActEEfs;
                     eef_id++) {
                    Eigen::Vector3d ctrl_rate_penalty = Eigen::Vector3d::Zero();
                    if (CentroidModel::isHand(eef_id)) {
                        ctrl_rate_penalty = mCentParam->wFrcArm;
                    } else {
                        ctrl_rate_penalty = mCentParam->wDFrcLeg;
                    }

                    if (mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id]) {
                        // current force
                        LinExpr current_force = mVars[mFrcWorld[eef_id].id(
                            axis_id, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id])];

                        // next force
                        LinExpr next_force = 0.0;
                        if (time_id == mCentParam->numTimeSteps - 1) {
                            next_force = current_force;
                        } else {
                            if (mDynStateSeq.dynamicsStateSequence[time_id + 1]
                                    .eEfsActivation[eef_id])
                                next_force = mVars[mFrcWorld[eef_id].id(
                                    axis_id,
                                    mDynStateSeq
                                        .dynamicsStateSequence[time_id + 1]
                                        .eEfsActivationIds[eef_id])];
                        }

                        // previous force
                        LinExpr previous_force = 0.0;
                        if (time_id == 0) {
                            previous_force = mCentParam->initialState
                                                 .eEfsFrc[eef_id][axis_id];
                        } else {
                            if (mDynStateSeq.dynamicsStateSequence[time_id - 1]
                                    .eEfsActivation[eef_id])
                                previous_force = mVars[mFrcWorld[eef_id].id(
                                    axis_id,
                                    mDynStateSeq
                                        .dynamicsStateSequence[time_id - 1]
                                        .eEfsActivationIds[eef_id])];
                        }

                        // penalty on force smoothness
                        mQuadObjective.addQuaTerm(ctrl_rate_penalty[axis_id],
                                                  next_force - current_force);
                        mQuadObjective.addQuaTerm(
                            ctrl_rate_penalty[axis_id],
                            current_force - previous_force);
                    }
                }
            }

            // penalty on torques
            for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsActivation[eef_id]) {
                    if (eef_id ==
                            static_cast<int>(CentroidModel::EEfID::rightFoot) ||
                        eef_id ==
                            static_cast<int>(CentroidModel::EEfID::leftFoot)) {
                        mQuadObjective.addQuaTerm(
                            mCentParam->wTrqLeg,
                            mVars[mTrqLocal[eef_id].id(
                                0, mDynStateSeq.dynamicsStateSequence[time_id]
                                       .eEfsActivationIds[eef_id])]);
                    } else {
                        mQuadObjective.addQuaTerm(
                            mCentParam->wTrqArm,
                            mVars[mTrqLocal[eef_id].id(
                                0, mDynStateSeq.dynamicsStateSequence[time_id]
                                       .eEfsActivationIds[eef_id])]);
                    }
                }
            }
        }

        mModel.setObjective(mQuadObjective, 0.0);

        // =====================================================================
        // upper and lower bounds constraints
        // =====================================================================
        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
            for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsActivation[eef_id]) {
                    mModel.addLinConstr(
                        mVars[mTrqLocal[eef_id].id(
                            0, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        ">", mCentParam->torqueRange[0]);
                    mModel.addLinConstr(
                        mVars[mTrqLocal[eef_id].id(
                            0, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        "<", mCentParam->torqueRange[1]);
                    mModel.addLinConstr(
                        mVars[mCopLocal[eef_id].id(
                            0, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        ">", mCentParam->copRange[eef_id][0]);
                    mModel.addLinConstr(
                        mVars[mCopLocal[eef_id].id(
                            0, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        "<", mCentParam->copRange[eef_id][1]);
                    mModel.addLinConstr(
                        mVars[mCopLocal[eef_id].id(
                            1, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        ">", mCentParam->copRange[eef_id][2]);
                    mModel.addLinConstr(
                        mVars[mCopLocal[eef_id].id(
                            1, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        "<", mCentParam->copRange[eef_id][3]);
                }
            }
        }

        // =====================================================================
        // friction cone constraints
        // =====================================================================
        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
            for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsContactType[eef_id] != ContactType::FullContact) {
                    if (mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id]) {
                        Eigen::Matrix3d eff_rotation =
                            mDynStateSeq.dynamicsStateSequence[time_id]
                                .eEfsOrientation[eef_id]
                                .toRotationMatrix();
                        if (mCentParam
                                ->isFrictionConeLinear) {  // using a linear
                                                           // representation
                            Eigen::Matrix<double, 4, 3> rotated_mConeMatrix =
                                mConeMatrix * eff_rotation.transpose();
                            for (int row_id = 0; row_id < 4; row_id++) {
                                mLinCons = 0.0;
                                for (int axis_id = 0; axis_id < 3; axis_id++)
                                    mLinCons +=
                                        LinExpr(mVars[mFrcWorld[eef_id].id(
                                            axis_id,
                                            mDynStateSeq
                                                .dynamicsStateSequence[time_id]
                                                .eEfsActivationIds[eef_id])]) *
                                        rotated_mConeMatrix(row_id, axis_id);
                                mModel.addLinConstr(mLinCons, "<", 0.0);
                            }
                        } else {  // using a second-order cone representation
                            LinExpr fx = 0.0, fy = 0.0, fz = 0.0;
                            for (int axis_id = 0; axis_id < 3; axis_id++) {
                                fx += LinExpr(mVars[mFrcWorld[eef_id].id(
                                          axis_id,
                                          mDynStateSeq
                                              .dynamicsStateSequence[time_id]
                                              .eEfsActivationIds[eef_id])]) *
                                      eff_rotation(axis_id, 0);
                                fy += LinExpr(mVars[mFrcWorld[eef_id].id(
                                          axis_id,
                                          mDynStateSeq
                                              .dynamicsStateSequence[time_id]
                                              .eEfsActivationIds[eef_id])]) *
                                      eff_rotation(axis_id, 1);
                                fz += LinExpr(mVars[mFrcWorld[eef_id].id(
                                          axis_id,
                                          mDynStateSeq
                                              .dynamicsStateSequence[time_id]
                                              .eEfsActivationIds[eef_id])]) *
                                      eff_rotation(axis_id, 2);
                            }
                            mQuadCons.clear();
                            mQuadCons.addQuaTerm(1.0, fx);
                            mQuadCons.addQuaTerm(1.0, fy);
                            mModel.addSocConstr(mQuadCons, "<",
                                                fz * mCentParam->frictionCoeff);
                        }
                    }
                }
            }
        }

        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
            // =================================================================
            // center of mass constraint
            // =================================================================
            for (int axis_id = 0; axis_id < 3; axis_id++) {
                if (time_id == 0) {
                    mLinCons =
                        LinExpr(mVars[mCom.id(axis_id, time_id)]) -
                        LinExpr(mCentParam->initialState.com[axis_id]) -
                        LinExpr(mVars[mLMom.id(axis_id, time_id)]) *
                            (mDynStateSeq.dynamicsStateSequence[time_id].time /
                             mCentParam->robotMass);
                } else {
                    mLinCons =
                        LinExpr(mVars[mCom.id(axis_id, time_id)]) -
                        LinExpr(mVars[mCom.id(axis_id, time_id - 1)]) -
                        LinExpr(mVars[mLMom.id(axis_id, time_id)]) *
                            (mDynStateSeq.dynamicsStateSequence[time_id].time /
                             mCentParam->robotMass);
                }
                mModel.addLinConstr(mLinCons, "=", 0.0);
            }

            // =================================================================
            // linear momentum constraint
            // =================================================================
            for (int axis_id = 0; axis_id < 3; axis_id++) {
                if (time_id == 0) {
                    mLinCons =
                        LinExpr(mVars[mLMom.id(axis_id, time_id)]) -
                        LinExpr(
                            mDynStateSeq.dynamicsStateSequence[time_id].time *
                            (mCentParam->robotMass *
                             mCentParam->gravityVector[axis_id])) -
                        LinExpr(mCentParam->initialState.lMom(axis_id));
                } else {
                    mLinCons =
                        LinExpr(mVars[mLMom.id(axis_id, time_id)]) -
                        LinExpr(
                            mDynStateSeq.dynamicsStateSequence[time_id].time *
                            (mCentParam->robotMass *
                             mCentParam->gravityVector[axis_id])) -
                        LinExpr(mVars[mLMom.id(axis_id, time_id - 1)]);
                }

                for (int eef_id = 0; eef_id < mCentParam->numActEEfs;
                     eef_id++) {
                    if (mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id]) {
                        mLinCons +=
                            LinExpr(mVars[mFrcWorld[eef_id].id(
                                axis_id,
                                mDynStateSeq.dynamicsStateSequence[time_id]
                                    .eEfsActivationIds[eef_id])]) *
                            (-mDynStateSeq.dynamicsStateSequence[time_id].time *
                             mCentParam->massTimesGravity);
                    }
                }
                if (time_id == 0) {
                    mLinCons +=
                        LinExpr(mCentParam->externalForce[axis_id]) *
                        (-mDynStateSeq.dynamicsStateSequence[time_id].time *
                         mCentParam->massTimesGravity);
                }
                mModel.addLinConstr(mLinCons, "=", 0.0);
            }

            // =================================================================
            // angular momentum constraint
            // =================================================================
            for (int axis_id = 0; axis_id < 3; axis_id++) {
                if (time_id == 0) {
                    mLinCons = LinExpr(mVars[mAMom.id(axis_id, time_id)]) -
                               LinExpr(mCentParam->initialState.aMom[axis_id]);
                } else {
                    mLinCons = LinExpr(mVars[mAMom.id(axis_id, time_id)]) -
                               LinExpr(mVars[mAMom.id(axis_id, time_id - 1)]);
                }

                for (int eef_id = 0; eef_id < mCentParam->numActEEfs;
                     eef_id++) {
                    if (mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id]) {
                        mLinCons +=
                            (LinExpr(mVars[mLbVar[eef_id].id(
                                 axis_id,
                                 mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsActivationIds[eef_id])]) -
                             LinExpr(mVars[mUbVar[eef_id].id(
                                 axis_id,
                                 mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsActivationIds[eef_id])])) *
                            (0.25 *
                             mDynStateSeq.dynamicsStateSequence[time_id].time *
                             mCentParam->massTimesGravity);

                        Eigen::Vector3d rz =
                            mDynStateSeq.dynamicsStateSequence[time_id]
                                .eEfsOrientation[eef_id]
                                .toRotationMatrix()
                                .col(2);
                        mLinCons +=
                            LinExpr(mVars[mTrqLocal[eef_id].id(
                                0, mDynStateSeq.dynamicsStateSequence[time_id]
                                       .eEfsActivationIds[eef_id])]) *
                            (-rz[axis_id] *
                             mDynStateSeq.dynamicsStateSequence[time_id].time);
                    }
                }
                mModel.addLinConstr(mLinCons, "=", 0.0);
            }
        }

        QuadConstrApprox qapprox = QuadConstrApprox::None;
        switch (mCentParam->heuristic) {
            case Heuristic::trustRegion: {
                qapprox = QuadConstrApprox::TrustRegion;
                break;
            }
            case Heuristic::softConstraint: {
                qapprox = QuadConstrApprox::SoftConstraint;
                break;
            }
            default: {
                qapprox = QuadConstrApprox::None;
                break;
            }
        }

        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
            for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsActivation[eef_id]) {
                    Eigen::Matrix3d rot =
                        mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsOrientation[eef_id]
                            .toRotationMatrix();
                    Eigen::Vector3d rx = rot.col(0);
                    Eigen::Vector3d ry = rot.col(1);

                    LinExpr lx, ly, lz, fx, fy, fz;
                    lx = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsPosition[eef_id]
                                     .x()) -
                         LinExpr(mVars[mCom.id(0, time_id)]) +
                         LinExpr(mVars[mCopLocal[eef_id].id(
                             0, mDynStateSeq.dynamicsStateSequence[time_id]
                                    .eEfsActivationIds[eef_id])]) *
                             rx(0) +
                         LinExpr(mVars[mCopLocal[eef_id].id(
                             1, mDynStateSeq.dynamicsStateSequence[time_id]
                                    .eEfsActivationIds[eef_id])]) *
                             ry(0);
                    ly = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsPosition[eef_id]
                                     .y()) -
                         LinExpr(mVars[mCom.id(1, time_id)]) +
                         LinExpr(mVars[mCopLocal[eef_id].id(
                             0, mDynStateSeq.dynamicsStateSequence[time_id]
                                    .eEfsActivationIds[eef_id])]) *
                             rx(1) +
                         LinExpr(mVars[mCopLocal[eef_id].id(
                             1, mDynStateSeq.dynamicsStateSequence[time_id]
                                    .eEfsActivationIds[eef_id])]) *
                             ry(1);
                    lz = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsPosition[eef_id]
                                     .z()) -
                         LinExpr(mVars[mCom.id(2, time_id)]) +
                         LinExpr(mVars[mCopLocal[eef_id].id(
                             0, mDynStateSeq.dynamicsStateSequence[time_id]
                                    .eEfsActivationIds[eef_id])]) *
                             rx(2) +
                         LinExpr(mVars[mCopLocal[eef_id].id(
                             1, mDynStateSeq.dynamicsStateSequence[time_id]
                                    .eEfsActivationIds[eef_id])]) *
                             ry(2);
                    fx = mVars[mFrcWorld[eef_id].id(
                        0, mDynStateSeq.dynamicsStateSequence[time_id]
                               .eEfsActivationIds[eef_id])];
                    fy = mVars[mFrcWorld[eef_id].id(
                        1, mDynStateSeq.dynamicsStateSequence[time_id]
                               .eEfsActivationIds[eef_id])];
                    fz = mVars[mFrcWorld[eef_id].id(
                        2, mDynStateSeq.dynamicsStateSequence[time_id]
                               .eEfsActivationIds[eef_id])];

                    // =========================================================
                    // upper and lower bounds on momentum rate constraints
                    // =========================================================
                    mQuadCons.clear();
                    mQuadCons.addQuaTerm(1.0, LinExpr() - lz + fy);
                    mQuadCons.addQuaTerm(1.0, ly + fz);
                    mModel.addQuaConstr(
                        mQuadCons, "<",
                        mVars[mUbVar[eef_id].id(
                            0, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        qapprox);

                    mQuadCons.clear();
                    mQuadCons.addQuaTerm(1.0, lz + fx);
                    mQuadCons.addQuaTerm(1.0, LinExpr() - lx + fz);
                    mModel.addQuaConstr(
                        mQuadCons, "<",
                        mVars[mUbVar[eef_id].id(
                            1, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        qapprox);

                    mQuadCons.clear();
                    mQuadCons.addQuaTerm(1.0, LinExpr() - ly + fx);
                    mQuadCons.addQuaTerm(1.0, lx + fy);
                    mModel.addQuaConstr(
                        mQuadCons, "<",
                        mVars[mUbVar[eef_id].id(
                            2, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        qapprox);

                    mQuadCons.clear();
                    mQuadCons.addQuaTerm(1.0, LinExpr() - lz - fy);
                    mQuadCons.addQuaTerm(1.0, ly - fz);
                    mModel.addQuaConstr(
                        mQuadCons, "<",
                        mVars[mLbVar[eef_id].id(
                            0, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        qapprox);

                    mQuadCons.clear();
                    mQuadCons.addQuaTerm(1.0, lz - fx);
                    mQuadCons.addQuaTerm(1.0, LinExpr() - lx - fz);
                    mModel.addQuaConstr(
                        mQuadCons, "<",
                        mVars[mLbVar[eef_id].id(
                            1, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        qapprox);

                    mQuadCons.clear();
                    mQuadCons.addQuaTerm(1.0, LinExpr() - ly - fx);
                    mQuadCons.addQuaTerm(1.0, lx - fy);
                    mModel.addQuaConstr(
                        mQuadCons, "<",
                        mVars[mLbVar[eef_id].id(
                            2, mDynStateSeq.dynamicsStateSequence[time_id]
                                   .eEfsActivationIds[eef_id])],
                        qapprox);

                    // =========================================================
                    // end-effector max length constraint
                    // =========================================================
                    mQuadCons.clear();
                    mQuadCons.addQuaTerm(
                        1.0, lx - mCentParam->eEfOffset[eef_id].x());
                    mQuadCons.addQuaTerm(
                        1.0, ly - mCentParam->eEfOffset[eef_id].y());
                    mQuadCons.addQuaTerm(
                        1.0, lz - mCentParam->eEfOffset[eef_id].z());
                    mModel.addQuaConstr(
                        mQuadCons, "<",
                        pow(mCentParam->maxEEfLengths[eef_id], 2.0));

                    // =========================================================
                    // end-effector min length constraint
                    // =========================================================
                    // mQuadCons.clear();
                    // mQuadCons.addQuaTerm(
                    // 1.0, lx - mCentParam->eEfOffset[eef_id].x());
                    // mQuadCons.addQuaTerm(
                    // 1.0, ly - mCentParam->eEfOffset[eef_id].y());
                    // mQuadCons.addQuaTerm(
                    // 1.0, lz - mCentParam->eEfOffset[eef_id].z());
                    // mModel.addQuaConstr(
                    // mQuadCons, ">",
                    // pow(mCentParam->minEEfLengths[eef_id], 2.0));
                }
            }
        }

        // formulate problem in standard conic form and solve it
        mTimer.start();
        mExitCode = mModel.optimize();
        mSolveTime += mTimer.stop();

        // extract solution
        mSolution.resize(mNumVars, 1);
        mSolution.setZero();
        for (int var_id = 0; var_id < mNumVars; var_id++) {
            mSolution(var_id) = mVars[var_id].get(SolverDoubleParam_X);
        }
    } catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }

    _saveSolution(mCom);
    _saveSolution(mAMom);
    _saveSolution(mLMom);
    for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
        _saveSolution(mFrcWorld[eef_id]);
        _saveSolution(mCopLocal[eef_id]);
        _saveSolution(mTrqLocal[eef_id]);
    }

    // computation of linear momentum out of forces
    Eigen::Vector3d frc, len, trq;
    Eigen::MatrixXd compos(3, mCentParam->numTimeSteps);
    compos.setZero();
    Eigen::MatrixXd linmom(3, mCentParam->numTimeSteps);
    linmom.setZero();
    Eigen::MatrixXd angmom(3, mCentParam->numTimeSteps);
    angmom.setZero();

    // computation of angular momentum out of forces and lengths
    for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
        if (time_id == 0) {
            angmom.col(time_id) = mCentParam->initialState.aMom;
        } else {
            angmom.col(time_id) = angmom.col(time_id - 1);
        }

        for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
            if (mDynStateSeq.dynamicsStateSequence[time_id]
                    .eEfsActivation[eef_id]) {
                Eigen::Matrix3d rot =
                    mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsOrientation[eef_id]
                        .toRotationMatrix();
                Eigen::Vector3d rx = rot.col(0);
                Eigen::Vector3d ry = rot.col(1);
                Eigen::Vector3d rz = rot.col(2);

                len.x() =
                    mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsPosition[eef_id]
                        .x() -
                    mVars[mCom.id(0, time_id)].get(SolverDoubleParam_X) +
                    rx(0) *
                        mVars[mCopLocal[eef_id].id(
                                  0, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id])]
                            .get(SolverDoubleParam_X) +
                    ry(0) *
                        mVars[mCopLocal[eef_id].id(
                                  1, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id])]
                            .get(SolverDoubleParam_X);
                len.y() =
                    mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsPosition[eef_id]
                        .y() -
                    mVars[mCom.id(1, time_id)].get(SolverDoubleParam_X) +
                    rx(1) *
                        mVars[mCopLocal[eef_id].id(
                                  0, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id])]
                            .get(SolverDoubleParam_X) +
                    ry(1) *
                        mVars[mCopLocal[eef_id].id(
                                  1, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id])]
                            .get(SolverDoubleParam_X);
                len.z() =
                    mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsPosition[eef_id]
                        .z() -
                    mVars[mCom.id(2, time_id)].get(SolverDoubleParam_X) +
                    rx(2) *
                        mVars[mCopLocal[eef_id].id(
                                  0, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id])]
                            .get(SolverDoubleParam_X) +
                    ry(2) *
                        mVars[mCopLocal[eef_id].id(
                                  1, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id])]
                            .get(SolverDoubleParam_X);
                frc.x() =
                    mCentParam->massTimesGravity *
                    mVars[mFrcWorld[eef_id].id(
                              0, mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsActivationIds[eef_id])]
                        .get(SolverDoubleParam_X);
                frc.y() =
                    mCentParam->massTimesGravity *
                    mVars[mFrcWorld[eef_id].id(
                              1, mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsActivationIds[eef_id])]
                        .get(SolverDoubleParam_X);
                frc.z() =
                    mCentParam->massTimesGravity *
                    mVars[mFrcWorld[eef_id].id(
                              2, mDynStateSeq.dynamicsStateSequence[time_id]
                                     .eEfsActivationIds[eef_id])]
                        .get(SolverDoubleParam_X);
                trq = rz *
                      mVars[mTrqLocal[eef_id].id(
                                0, mDynStateSeq.dynamicsStateSequence[time_id]
                                       .eEfsActivationIds[eef_id])]
                          .get(SolverDoubleParam_X);
                angmom.col(time_id).head(3) +=
                    mDynStateSeq.dynamicsStateSequence[time_id].time *
                    (len.cross(frc) + trq);
            }
        }
    }
    if (mCentParam->heuristic != Heuristic::timeOptimization) {
        mAMom.setGuessValue(angmom);
    }
    _storeSolution();
}

void CentroidPlanner::GetSolution(
    Eigen::MatrixXd& com, Eigen::MatrixXd& lmom, Eigen::MatrixXd& amom,
    std::array<Eigen::MatrixXd, CentroidModel::numEEf>& cop,
    std::array<Eigen::MatrixXd, CentroidModel::numEEf>& frc,
    std::array<Eigen::MatrixXd, CentroidModel::numEEf>& trq) {
    mCom.getGuessValue(com);
    mLMom.getGuessValue(lmom);
    mAMom.getGuessValue(amom);
    for (int eef_id = 0; eef_id < mCentParam->numActEEfs; ++eef_id) {
        mCopLocal[eef_id].getGuessValue(cop[eef_id]);
        mFrcWorld[eef_id].getGuessValue(frc[eef_id]);
        mTrqLocal[eef_id].getGuessValue(trq[eef_id]);
    }
}

void CentroidPlanner::SaveResult(const std::string& file_name) {
    try {
        YAML::Node qcqp_cfg;
        qcqp_cfg["dynopt_params"]["time_step"] = mCentParam->timeStep;
        qcqp_cfg["dynopt_params"]["end_com"] = mComPosGoal;
        qcqp_cfg["dynopt_params"]["robot_mass"] = mCentParam->robotMass;
        qcqp_cfg["dynopt_params"]["n_act_eefs"] = mCentParam->numActEEfs;
        qcqp_cfg["dynopt_params"]["ini_com"] = mCentParam->initialState.com;
        qcqp_cfg["dynopt_params"]["ini_ang_mom"] =
            mCentParam->initialState.aMom;
        qcqp_cfg["dynopt_params"]["ini_lin_mom"] =
            mCentParam->initialState.lMom;
        qcqp_cfg["dynopt_params"]["time_horizon"] = mCentParam->timeHorizon;
        qcqp_cfg["cntopt_params"] = mCentParam->contact_plan;

        mCom.getGuessValue(mMatGuess);
        qcqp_cfg["dynopt_params"]["com_motion"] = mMatGuess;
        mLMom.getGuessValue(mMatGuess);
        qcqp_cfg["dynopt_params"]["lin_mom"] = mMatGuess;
        mAMom.getGuessValue(mMatGuess);
        qcqp_cfg["dynopt_params"]["ang_mom"] = mMatGuess;

        // building momentum references
        // for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
        // { mMatGuess.col(time_id) =
        //_ref_sequence.dynamicsStateSequence[time_id].com;
        //}
        // qcqp_cfg["dynopt_params"]["com_motion_ref"] = mMatGuess;
        // for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
        // { mMatGuess.col(time_id) =
        //_ref_sequence.dynamicsStateSequence[time_id].lMom;
        //}
        // qcqp_cfg["dynopt_params"]["lin_mom_ref"] = mMatGuess;
        // for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
        // { mMatGuess.col(time_id) =
        //_ref_sequence.dynamicsStateSequence[time_id].aMom;
        //}
        // qcqp_cfg["dynopt_params"]["ang_mom_ref"] = mMatGuess;

        // saving vector of time-steps
        mMatGuess.resize(1, mCentParam->numTimeSteps);
        mMatGuess.setZero();
        mMatGuess(0, 0) = mDynStateSeq.dynamicsStateSequence[0].time;
        for (int time_id = 1; time_id < mCentParam->numTimeSteps; time_id++)
            mMatGuess(0, time_id) =
                mMatGuess(0, time_id - 1) +
                mDynStateSeq.dynamicsStateSequence[time_id].time;
        qcqp_cfg["dynopt_params"]["time_vec"] = mMatGuess;

        // saving vector of forces, torques and cops
        for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
            mMatGuess.resize(3, mCentParam->numTimeSteps);
            mMatGuess.setZero();
            for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
                if (mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsActivation[eef_id])
                    mMatGuess.col(time_id).head(3) =
                        mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsFrc[eef_id];
            qcqp_cfg["dynopt_params"]["eef_frc_" + std::to_string(eef_id)] =
                mMatGuess;

            mMatGuess.resize(1, mCentParam->numTimeSteps);
            mMatGuess.setZero();
            for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
                if (mDynStateSeq.dynamicsStateSequence[time_id]
                        .eEfsActivation[eef_id])
                    mMatGuess(0, time_id) =
                        mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsTrq[eef_id]
                            .z();
            qcqp_cfg["dynopt_params"]["eef_trq_" + std::to_string(eef_id)] =
                mMatGuess;

            mCopLocal[eef_id].getGuessValue(mMatGuess);
            qcqp_cfg["dynopt_params"]["eef_cop_" + std::to_string(eef_id)] =
                mMatGuess;
        }
        std::string full_path = THIS_COM + std::string("ExperimentData/") +
                                file_name + std::string(".yaml");
        std::ofstream file_out(full_path);
        file_out << qcqp_cfg;
    } catch (YAML::ParserException& e) {
        std::cout << e.what() << "\n";
    }
}

void CentroidPlanner::_saveToFile(const DynamicsStateSequence& _ref_sequence) {
    if (mCentParam->isStoreData) {
        try {
            YAML::Node qcqp_cfg;
            qcqp_cfg["dynopt_params"]["time_step"] = mCentParam->timeStep;
            qcqp_cfg["dynopt_params"]["end_com"] = mComPosGoal;
            qcqp_cfg["dynopt_params"]["robot_mass"] = mCentParam->robotMass;
            qcqp_cfg["dynopt_params"]["n_act_eefs"] = mCentParam->numActEEfs;
            qcqp_cfg["dynopt_params"]["ini_com"] = mCentParam->initialState.com;
            qcqp_cfg["dynopt_params"]["ini_ang_mom"] =
                mCentParam->initialState.aMom;
            qcqp_cfg["dynopt_params"]["ini_lin_mom"] =
                mCentParam->initialState.lMom;
            qcqp_cfg["dynopt_params"]["time_horizon"] = mCentParam->timeHorizon;
            qcqp_cfg["cntopt_params"] = mCentParam->contact_plan;

            mCom.getGuessValue(mMatGuess);
            qcqp_cfg["dynopt_params"]["com_motion"] = mMatGuess;
            mLMom.getGuessValue(mMatGuess);
            qcqp_cfg["dynopt_params"]["lin_mom"] = mMatGuess;
            mAMom.getGuessValue(mMatGuess);
            qcqp_cfg["dynopt_params"]["ang_mom"] = mMatGuess;

            // building momentum references
            for (int time_id = 0; time_id < mCentParam->numTimeSteps;
                 time_id++) {
                mMatGuess.col(time_id) =
                    _ref_sequence.dynamicsStateSequence[time_id].com;
            }
            qcqp_cfg["dynopt_params"]["com_motion_ref"] = mMatGuess;
            for (int time_id = 0; time_id < mCentParam->numTimeSteps;
                 time_id++) {
                mMatGuess.col(time_id) =
                    _ref_sequence.dynamicsStateSequence[time_id].lMom;
            }
            qcqp_cfg["dynopt_params"]["lin_mom_ref"] = mMatGuess;
            for (int time_id = 0; time_id < mCentParam->numTimeSteps;
                 time_id++) {
                mMatGuess.col(time_id) =
                    _ref_sequence.dynamicsStateSequence[time_id].aMom;
            }
            qcqp_cfg["dynopt_params"]["ang_mom_ref"] = mMatGuess;

            // saving vector of time-steps
            mMatGuess.resize(1, mCentParam->numTimeSteps);
            mMatGuess.setZero();
            mMatGuess(0, 0) = mDynStateSeq.dynamicsStateSequence[0].time;
            for (int time_id = 1; time_id < mCentParam->numTimeSteps; time_id++)
                mMatGuess(0, time_id) =
                    mMatGuess(0, time_id - 1) +
                    mDynStateSeq.dynamicsStateSequence[time_id].time;
            qcqp_cfg["dynopt_params"]["time_vec"] = mMatGuess;

            // saving vector of forces, torques and cops
            for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                mMatGuess.resize(3, mCentParam->numTimeSteps);
                mMatGuess.setZero();
                for (int time_id = 0; time_id < mCentParam->numTimeSteps;
                     time_id++)
                    if (mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id])
                        mMatGuess.col(time_id).head(3) =
                            mDynStateSeq.dynamicsStateSequence[time_id]
                                .eEfsFrc[eef_id];
                qcqp_cfg["dynopt_params"]["eef_frc_" + std::to_string(eef_id)] =
                    mMatGuess;

                mMatGuess.resize(1, mCentParam->numTimeSteps);
                mMatGuess.setZero();
                for (int time_id = 0; time_id < mCentParam->numTimeSteps;
                     time_id++)
                    if (mDynStateSeq.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id])
                        mMatGuess(0, time_id) =
                            mDynStateSeq.dynamicsStateSequence[time_id]
                                .eEfsTrq[eef_id]
                                .z();
                qcqp_cfg["dynopt_params"]["eef_trq_" + std::to_string(eef_id)] =
                    mMatGuess;

                mCopLocal[eef_id].getGuessValue(mMatGuess);
                qcqp_cfg["dynopt_params"]["eef_cop_" + std::to_string(eef_id)] =
                    mMatGuess;
            }
            std::ofstream file_out(mCentParam->saveDynamicsFile);
            file_out << qcqp_cfg;
        } catch (YAML::ParserException& e) {
            std::cout << e.what() << "\n";
        }
    }
}

void CentroidPlanner::_saveSolution(solver::OptimizationVariable& opt_var) {
    mMatGuess.resize(opt_var.getNumRows(), opt_var.getNumCols());
    mMatGuess.setZero();
    for (int col_id = 0; col_id < opt_var.getNumCols(); col_id++) {
        for (int row_id = 0; row_id < opt_var.getNumRows(); row_id++)
            mMatGuess(row_id, col_id) = mSolution(opt_var.id(row_id, col_id));
    }
    opt_var.setGuessValue(mMatGuess);
}

void CentroidPlanner::_storeSolution() {
    mCom.getGuessValue(mMatGuess);
    for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].com =
            Eigen::Vector3d(mMatGuess.block<3, 1>(0, time_id));

    mLMom.getGuessValue(mMatGuess);
    for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].lMom =
            Eigen::Vector3d(mMatGuess.block<3, 1>(0, time_id));

    mAMom.getGuessValue(mMatGuess);
    for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].aMom =
            Eigen::Vector3d(mMatGuess.block<3, 1>(0, time_id));

    for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
        mFrcWorld[eef_id].getGuessValue(mMatGuess);
        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
            if (mDynStateSeq.dynamicsStateSequence[time_id]
                    .eEfsActivation[eef_id])
                mDynStateSeq.dynamicsStateSequence[time_id].eEfsFrc[eef_id] =
                    Eigen::Vector3d(mMatGuess.block<3, 1>(
                        0, mDynStateSeq.dynamicsStateSequence[time_id]
                               .eEfsActivationIds[eef_id]));

        mCopLocal[eef_id].getGuessValue(mMatGuess);
        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
            if (mDynStateSeq.dynamicsStateSequence[time_id]
                    .eEfsActivation[eef_id])
                mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id] =
                    Eigen::Vector3d(
                        mMatGuess(0, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id]),
                        mMatGuess(1, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id]),
                        0.0);

        mTrqLocal[eef_id].getGuessValue(mMatGuess);
        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++)
            if (mDynStateSeq.dynamicsStateSequence[time_id]
                    .eEfsActivation[eef_id])
                mDynStateSeq.dynamicsStateSequence[time_id].eEfsTrq[eef_id] =
                    Eigen::Vector3d(
                        0.0, 0.0,
                        mMatGuess(0, mDynStateSeq.dynamicsStateSequence[time_id]
                                         .eEfsActivationIds[eef_id]));
    }
}

void CentroidPlanner::_addVariableToModel(const OptimizationVariable& opt_var,
                                          Model& model,
                                          std::vector<Var>& vars) {
    opt_var.getValues(mMatLb, mMatUb, mMatGuess, mSize, mVariableType);
    for (int col_id = 0; col_id < opt_var.getNumCols(); col_id++)
        for (int row_id = 0; row_id < opt_var.getNumRows(); row_id++)
            switch (mVariableType) {
                case 'C': {
                    vars[opt_var.id(row_id, col_id)] = model.addVar(
                        VarType::Continuous, double(mMatLb(row_id, col_id)),
                        double(mMatUb(row_id, col_id)),
                        double(mMatGuess(row_id, col_id)));
                    break;
                }
                default: {
                    throw std::runtime_error(
                        "At add_var_to_model, variable type not handled");
                }
            }

    for (int col_id = 0; col_id < opt_var.getNumCols(); col_id++)
        for (int row_id = 0; row_id < opt_var.getNumRows(); row_id++)
            vars[opt_var.id(row_id, col_id)].set(SolverDoubleParam_X,
                                                 mMatGuess(row_id, col_id));
}
