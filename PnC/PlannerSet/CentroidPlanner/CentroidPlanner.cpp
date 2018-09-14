#include "CentroidPlanner.hpp"
#include "Utilities.hpp"

using namespace myUtils;
using namespace solver;

void CentroidPlannerParameter::paramSetFromYaml(const std::string & cfg_file) {
    cfgFile = cfg_file;
    saveDynamicsFile = cfg_file.substr(0, cfg_file.size()-5) + "_RESULT.yaml";
    defaultSolverSettingFile = THIS_COM + std::string("Config/Solver/DEFAULT_CONIC_SOLVER_SETTING.yaml");
    try {
        YAML::Node planner_cfg = YAML::LoadFile(cfgFile.c_str());

        // ===============
        // Planner Setting
        // ===============
        YAML::Node planner_vars = planner_cfg["planner_variables"];
        // Dynamics parameters
        readParameter(planner_vars, "num_com_viapoints", numComViaPoints);
        comViaPoints.clear();
        for (int via_id=0; via_id<numComViaPoints; via_id++) {
            comViaPoints.push_back(Eigen::Vector4d::Zero());
            readParameter(planner_vars["com_viapoints"], "via"+std::to_string(via_id), comViaPoints[via_id]);
        }

        readParameter(planner_vars, "time_step", timeStep);
        readParameter(planner_vars, "n_act_eefs", numActEEfs);
        readParameter(planner_vars, "time_horizon", timeHorizon);
        readParameter(planner_vars, "external_force", externalForce);
        readParameter(planner_vars, "com_displacement", comDisplacement);
        if (readParameter<std::string>(planner_vars, "heuristic").compare("TrustRegion")==0) { heuristic = Heuristic::trustRegion; }
        else if (readParameter<std::string>(planner_vars, "heuristic").compare("SoftConstraint")==0) { heuristic = Heuristic::softConstraint; }
        else if (readParameter<std::string>(planner_vars, "heuristic").compare("TimeOptimization")==0) { heuristic = Heuristic::timeOptimization; }
        else { heuristic = Heuristic::softConstraint; }
        // Time optimization parameters
        if (heuristic == Heuristic::timeOptimization) {
            readParameter(planner_vars, "max_time_iterations", maxTimeIterations);
            readParameter(planner_vars, "max_time_residual_tolerance", maxTimeResidualTolerance);
            readParameter(planner_vars, "min_time_residual_improvement", minTimeResidualImprovement);
        }
        // Configuration parameters
        readParameter(planner_vars, "gravity", gravity);
        readParameter(planner_vars, "robot_mass", robotMass);
        readParameter(planner_vars, "torque_range", torqueRange);
        readParameter(planner_vars, "friction_coeff", frictionCoeff);
        readParameter(planner_vars, "max_eef_lengths", maxEEfLengths);
        if (heuristic == Heuristic::timeOptimization) {
            readParameter(planner_vars, "time_range", timeRange);
            readParameter(planner_vars, "is_time_horizon_fixed", isTimeHorizonFixed);
        }
        for (int eef_id=0; eef_id<CentroidModel::numEEf; eef_id++) {
            readParameter(planner_vars, "cop_range_"+CentroidModel::eEfIdToString(eef_id), copRange[eef_id]);
            readParameter(planner_vars, "eef_offset_"+CentroidModel::eEfIdToString(eef_id), eEfOffset[eef_id]);
        }
        isFrictionConeLinear = (readParameter<std::string>(planner_vars, "friction_cone").compare("LinearCone")==0);
        // Dynamics weights
        readParameter(planner_vars, "w_com", wCom);
        readParameter(planner_vars, "w_amom", wAMom);
        readParameter(planner_vars, "w_lmom", wLMom);
        readParameter(planner_vars, "w_amomd", wAMomD);
        readParameter(planner_vars, "w_lmomd", wLMomD);
        readParameter(planner_vars, "w_com_via", wComVia);
        readParameter(planner_vars, "w_trq_arm", wTrqArm);
        readParameter(planner_vars, "w_trq_leg", wTrqLeg);
        readParameter(planner_vars, "w_frc_arm", wFrcArm);
        readParameter(planner_vars, "w_frc_leg", wFrcLeg);
        readParameter(planner_vars, "w_dfrc_arm", wDFrcArm);
        readParameter(planner_vars, "w_dfrc_leg", wDFrcLeg);
        readParameter(planner_vars, "w_amom_final", wAMomFinal);
        readParameter(planner_vars, "w_lmom_final", wLMomFinal);
        readParameter(planner_vars, "w_amom_track", wAMomTrack);
        readParameter(planner_vars, "w_lmom_track", wLMomTrack);
        if (heuristic == Heuristic::timeOptimization) {
            readParameter(planner_vars, "w_time", wTime);
            readParameter(planner_vars, "w_time_penalty", wTimePenalty);
        }
        // Storage information
        readParameter(planner_vars, "store_data", isStoreData);
        // Solver setting
        readParameter(planner_vars, "use_default_solver_setting", isDefaultSolverSetting);
        massTimesGravity = robotMass * gravity;
        gravityVector = Eigen::Vector3d(0., 0., -gravity);
        numTimeSteps = std::floor(timeHorizon/timeStep);

        // ===================
        // Robot Initial State
        // ===================
        YAML::Node ini_robo_cfg = planner_cfg["initial_robot_configuration"];
        readParameter(ini_robo_cfg, "com", initialState.com);
        readParameter(ini_robo_cfg, "lmom", initialState.lMom);
        readParameter(ini_robo_cfg, "amom", initialState.aMom);
        for (int eef_id=0; eef_id<CentroidModel::numEEf; eef_id++) {
            Eigen::VectorXd eef_cfg = readParameter<Eigen::VectorXd>(ini_robo_cfg["eef_pose"], "eef_"+CentroidModel::eEfIdToString(eef_id));
            initialState.eEfsActivation[eef_id] = int(eef_cfg(0));
            initialState.eEfsPosition[eef_id] = eef_cfg.segment<3>(1);
            initialState.eEfsOrientation[eef_id] = Eigen::Quaternion<double>(eef_cfg[4],eef_cfg[5],eef_cfg[6],eef_cfg[7]);
            readParameter(ini_robo_cfg["eef_ctrl"], "eef_frc_"+CentroidModel::eEfIdToString(eef_id), initialState.eEfsFrc[eef_id]);
        }

        // ===========================
        // Reference Dynamics Sequence
        // ===========================
        refDynamicsStateSequence.resize(numTimeSteps);

        // ================
        // Contact Interface
        // ================
        YAML::Node contact_vars = planner_cfg["contact_plan"];

        // Contact parameters
        readParameter(contact_vars, "num_contacts", contactPlanInterface.contactsPerEndeff);
        for (int eef_id=0; eef_id<CentroidModel::numEEf; eef_id++) {
            contactPlanInterface.contactSequence.eEfContacts[eef_id].clear();
            if (contactPlanInterface.contactsPerEndeff[eef_id]>0) {
                YAML::Node eff_params = contact_vars[("eefcnt_" + CentroidModel::eEfIdToString(eef_id)).c_str()];
                for (int cnt_id=0; cnt_id<contactPlanInterface.contactsPerEndeff[eef_id]; cnt_id++) {
                    Eigen::VectorXd v(10);
                    readParameter(eff_params, "cnt"+std::to_string(cnt_id), v);
                    contactPlanInterface.contactSequence.eEfContacts[eef_id].push_back(ContactState());
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeIni = v[0];
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].timeEnd = v[1];
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].position = v.segment<3>(2);
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].contactType = idToContactType(v(9));
                    contactPlanInterface.contactSequence.eEfContacts[eef_id][cnt_id].orientation = Eigen::Quaternion<double>(v[5],v[6],v[7],v[8]);
                }
            }
        }

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
}


//CentroidPlanner::CentroidPlanner() : Planner(){
CentroidPlanner::CentroidPlanner() : Planner(){
}

CentroidPlanner::~CentroidPlanner() {}

void CentroidPlanner::_doPlan() {
    mCentParam = std::dynamic_pointer_cast<CentroidPlannerParameter>(mParam);
    _initialize();
    _optimize();
}

void CentroidPlanner::_evalTrajectory(double time,
        Eigen::VectorXd & pos,
        Eigen::VectorXd & vel,
        Eigen::VectorXd &trq) {

}

void CentroidPlanner::_initialize() {
    if (!mCentParam->isDefaultSolverSetting) {
        mModel.configSetting(mCentParam->cfgFile);
    } else {
        mModel.configSetting(mCentParam->defaultSolverSettingFile);
    }

    mComPosGoal = mCentParam->initialState.com + mCentParam->comDisplacement;
    mFrictionCone.getCone(mCentParam->frictionCoeff, mConeMatrix);
    mDynStateSeq.resize(mCentParam->numTimeSteps);
    for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].time = mCentParam->timeStep;

    mCentParam->contactPlanInterface.fillDynamicsSequence(mDynStateSeq,
                                                          mCentParam->numTimeSteps,
                                                          mCentParam->timeStep);

    _initializeOptimizationVariables();
}

void CentroidPlanner::_initializeOptimizationVariables() {
    mNumVars = 0.;
    double inf_value = SolverSetting::inf;

    // center of mass, linear and angular momentum
    mCom.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value, mNumVars);
    mLMom.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value, mNumVars);
    mAMom.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value, mNumVars);

    // time variable, linear and angular momentum rates
    if (mCentParam->heuristic == Heuristic::timeOptimization) {
        mDt.initialize('C', 1, mCentParam->numTimeSteps, -inf_value, inf_value, mNumVars);
        mLMomD.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value, mNumVars);
        mAMomD.initialize('C', 3, mCentParam->numTimeSteps, -inf_value, inf_value, mNumVars);
    }

    // upper and lower bound variables, forces, cops, torques
    for (int eef_id=0; eef_id < mCentParam->numActEEfs; eef_id++) {
        mLbVar[eef_id].initialize('C', 3, mDynStateSeq.activeEEfSteps[eef_id], -inf_value, inf_value, mNumVars);
        mUbVar[eef_id].initialize('C', 3, mDynStateSeq.activeEEfSteps[eef_id], -inf_value, inf_value, mNumVars);
        mFrcWorld[eef_id].initialize('C', 3, mDynStateSeq.activeEEfSteps[eef_id], -inf_value, inf_value, mNumVars);
        mCopLocal[eef_id].initialize('C', 2, mDynStateSeq.activeEEfSteps[eef_id], -inf_value, inf_value, mNumVars);
        mTrqLocal[eef_id].initialize('C', 1, mDynStateSeq.activeEEfSteps[eef_id], -inf_value, inf_value, mNumVars);
    }
}

solver::ExitCode CentroidPlanner::_optimize(bool update_tracking_objective) {
    if (update_tracking_objective) _updateTrackingObjective();

    mSolveTime = 0.0;
    mHasConverged = false;
    _internalOptimize(true);

    if (mCentParam->heuristic == Heuristic::timeOptimization) {
        for (int iter_id=1; iter_id <= mCentParam->maxTimeIterations; iter_id++) {
            _internalOptimize();
            if (mHasConverged) break;
        }
    }

    _saveToFile(mCentParam->refDynamicsStateSequence);
    return mExitCode;
}

void CentroidPlanner::_updateTrackingObjective() {
    mCentParam->wLMom = mCentParam->wLMomTrack;
    mCentParam->wAMom = mCentParam->wAMomTrack;
}

void CentroidPlanner::_internalOptimize(bool is_first_time) {
    try
    {
        // add variables to model
        mVars.clear();
        for (int var_id=0; var_id<mNumVars; var_id++)
            mVars.push_back(Var());

        mModel.clean();
        _addVariableToModel(mCom, mModel, mVars);
        _addVariableToModel(mLMom, mModel, mVars);
        _addVariableToModel(mAMom, mModel, mVars);

        if (mCentParam->heuristic == Heuristic::timeOptimization) {
            _addVariableToModel(mDt, mModel, mVars);
            _addVariableToModel(mLMomD, mModel, mVars);
            _addVariableToModel(mAMomD, mModel, mVars);
        }

        for (int eef_id=0; eef_id < mCentParam->numActEEfs; eef_id++) {
            _addVariableToModel(mLbVar[eef_id], mModel, mVars);
            _addVariableToModel(mUbVar[eef_id], mModel, mVars);
            _addVariableToModel(mFrcWorld[eef_id], mModel, mVars);
            _addVariableToModel(mCopLocal[eef_id], mModel, mVars);
            _addVariableToModel(mTrqLocal[eef_id], mModel, mVars);
        }

        // adding quadratic objective
        mQuadObjective.clear();

        if (mCentParam->numComViaPoints > 0) {
            for (int via_id=0; via_id < mCentParam->numComViaPoints; via_id++)
                for (int axis_id=0; axis_id<3; axis_id++)
                    mQuadObjective.addQuaTerm(mCentParam->wComVia[axis_id], LinExpr(mVars[mCom.id(axis_id,int(mCentParam->comViaPoints[via_id](0)/mCentParam->timeStep))]) - LinExpr(mCentParam->comViaPoints[via_id](axis_id+1)) );
        }

        for (int time_id=0; time_id < mCentParam->numTimeSteps; time_id++) {
            for (int axis_id=0; axis_id<3; axis_id++) {

                // penalty on center of mass, linear and angular momentum
                if (time_id == mCentParam->numTimeSteps-1) {
                    mQuadObjective.addQuaTerm(mCentParam->wCom[axis_id], LinExpr(mVars[mCom.id(axis_id,time_id)]) - LinExpr(mComPosGoal[axis_id]));
                    mQuadObjective.addQuaTerm(mCentParam->wLMomFinal[axis_id], LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mCentParam->refDynamicsStateSequence.dynamicsStateSequence[time_id].lMom[axis_id]));
                    mQuadObjective.addQuaTerm(mCentParam->wAMomFinal[axis_id], LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mCentParam->refDynamicsStateSequence.dynamicsStateSequence[time_id].aMom[axis_id]));
                } else {
                    mQuadObjective.addQuaTerm(mCentParam->wLMom[axis_id], LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mCentParam->refDynamicsStateSequence.dynamicsStateSequence[time_id].lMom[axis_id]));
                    mQuadObjective.addQuaTerm(mCentParam->wAMom[axis_id], LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mCentParam->refDynamicsStateSequence.dynamicsStateSequence[time_id].aMom[axis_id]));
                }

                // penalty on linear and angular momentum rates
                if (mCentParam->heuristic == Heuristic::timeOptimization) {
                    mQuadObjective.addQuaTerm(mCentParam->wLMomD[axis_id], mVars[mLMomD.id(axis_id,time_id)]);
                    mQuadObjective.addQuaTerm(mCentParam->wAMomD[axis_id], mVars[mAMomD.id(axis_id,time_id)]);
                } else {
                    if (time_id==0) {
                        mQuadObjective.addQuaTerm(mCentParam->wLMomD[axis_id], (LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mCentParam->initialState.lMom[axis_id]))*(1.0/mDynStateSeq.dynamicsStateSequence[time_id].time));
                        mQuadObjective.addQuaTerm(mCentParam->wAMomD[axis_id], (LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mCentParam->initialState.aMom[axis_id]))*(1.0/mDynStateSeq.dynamicsStateSequence[time_id].time));
                    } else {
                        mQuadObjective.addQuaTerm(mCentParam->wLMomD[axis_id], (LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mVars[mLMom.id(axis_id,time_id-1)]))*(1.0/mDynStateSeq.dynamicsStateSequence[time_id].time));
                        mQuadObjective.addQuaTerm(mCentParam->wAMomD[axis_id], (LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mVars[mAMom.id(axis_id,time_id-1)]))*(1.0/mDynStateSeq.dynamicsStateSequence[time_id].time));
                    }
                }

                // penalty on forces
                for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
                    if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                        if (eef_id==static_cast<int>(CentroidModel::EEfID::rightFoot) || eef_id==static_cast<int>(CentroidModel::EEfID::leftFoot)) {
                            mQuadObjective.addQuaTerm(mCentParam->wFrcLeg[axis_id], mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                        } else {
                            mQuadObjective.addQuaTerm(mCentParam->wFrcArm[axis_id], mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                        }
                    }
                }

                // penalty on rate of forces
                for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                    Eigen::Vector3d ctrl_rate_penalty = Eigen::Vector3d::Zero();
                    if (CentroidModel::isHand(eef_id)) { ctrl_rate_penalty = mCentParam->wFrcArm; }
                    else { ctrl_rate_penalty = mCentParam->wDFrcLeg; }

                    if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                        // current force
                        LinExpr current_force = mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])];

                        // next force
                        LinExpr next_force = 0.0;
                        if (time_id == mCentParam->numTimeSteps-1) { next_force = current_force; }
                        else {
                            if (mDynStateSeq.dynamicsStateSequence[time_id+1].eEfsActivation[eef_id])
                                next_force = mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id+1].eEfsActivationIds[eef_id])];
                        }

                        // previous force
                        LinExpr previous_force = 0.0;
                        if (time_id==0) { previous_force = mCentParam->initialState.eEfsFrc[eef_id][axis_id]; }
                        else {
                            if (mDynStateSeq.dynamicsStateSequence[time_id-1].eEfsActivation[eef_id])
                                previous_force = mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id-1].eEfsActivationIds[eef_id])];
                        }

                        // penalty on force smoothness
                        mQuadObjective.addQuaTerm(ctrl_rate_penalty[axis_id], next_force - current_force);
                        mQuadObjective.addQuaTerm(ctrl_rate_penalty[axis_id], current_force - previous_force);
                    }
                }
            }

            // penalty on torques
            for (int eef_id=0; eef_id < mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                    if (eef_id == static_cast<int>(CentroidModel::EEfID::rightFoot) || eef_id==static_cast<int>(CentroidModel::EEfID::leftFoot)) {
                        mQuadObjective.addQuaTerm(mCentParam->wTrqLeg, mVars[mTrqLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                    }
                    else {
                        mQuadObjective.addQuaTerm(mCentParam->wTrqArm, mVars[mTrqLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                    }
                }
            }

            // penalty on time
            if (mCentParam->heuristic == Heuristic::timeOptimization) {
                mQuadObjective.addQuaTerm(mCentParam->wTimePenalty, LinExpr(mVars[mDt.id(0,time_id)]));
                mQuadObjective.addQuaTerm(mCentParam->wTime, LinExpr(mVars[mDt.id(0,time_id)]) - LinExpr(mCentParam->timeStep));
            }
        }

        if (!is_first_time && mCentParam->heuristic == Heuristic::timeOptimization) {
            for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
                for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                    if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                        Eigen::Matrix3d rot = mDynStateSeq.dynamicsStateSequence[time_id].eEfsOrientation[eef_id].toRotationMatrix();
                        Eigen::Vector3d rx = rot.col(0);
                        Eigen::Vector3d ry = rot.col(1);

                        LinExpr lx, ly, lz, fx, fy, fz;
                        lx = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].x()) - LinExpr(mVars[mCom.id(0,time_id)]) + LinExpr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rx(0) + LinExpr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*ry(0);
                        ly = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].y()) - LinExpr(mVars[mCom.id(1,time_id)]) + LinExpr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rx(1) + LinExpr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*ry(1);
                        lz = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].z()) - LinExpr(mVars[mCom.id(2,time_id)]) + LinExpr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rx(2) + LinExpr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*ry(2);
                        fx = mVars[mFrcWorld[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])];
                        fy = mVars[mFrcWorld[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])];
                        fz = mVars[mFrcWorld[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])];

                        double lx_val, ly_val, lz_val, fx_val, fy_val, fz_val;
                        lx_val = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].x() - mDynStateSeq.dynamicsStateSequence[time_id].com.x() + rx(0)*mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id].x() + ry(0)*mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id].y();
                        ly_val = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].y() - mDynStateSeq.dynamicsStateSequence[time_id].com.y() + rx(1)*mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id].x() + ry(1)*mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id].y();
                        lz_val = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].z() - mDynStateSeq.dynamicsStateSequence[time_id].com.z() + rx(2)*mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id].x() + ry(2)*mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id].y();
                        fx_val = mDynStateSeq.dynamicsStateSequence[time_id].eEfsFrc[eef_id].x();
                        fy_val = mDynStateSeq.dynamicsStateSequence[time_id].eEfsFrc[eef_id].y();
                        fz_val = mDynStateSeq.dynamicsStateSequence[time_id].eEfsFrc[eef_id].z();

                        LinExpr ub_x, lb_x, ub_y, lb_y, ub_z, lb_z;
                        ub_x = ( LinExpr(std::pow(-lz_val+fy_val, 2.0) + std::pow( ly_val+fz_val, 2.0)) + ((LinExpr()-lz+fy)-(-lz_val+fy_val))*2.0*(-lz_val+fy_val) + (( ly+fz)-( ly_val+fz_val))*2.0*( ly_val+fz_val) ) - LinExpr(mVars[mUbVar[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                        lb_x = ( LinExpr(std::pow(-lz_val-fy_val, 2.0) + std::pow( ly_val-fz_val, 2.0)) + ((LinExpr()-lz-fy)-(-lz_val-fy_val))*2.0*(-lz_val-fy_val) + (( ly-fz)-( ly_val-fz_val))*2.0*( ly_val-fz_val) ) - LinExpr(mVars[mLbVar[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                        ub_y = ( LinExpr(std::pow( lz_val+fx_val, 2.0) + std::pow(-lx_val+fz_val, 2.0)) + (( lz+fx)-( lz_val+fx_val))*2.0*( lz_val+fx_val) + ((LinExpr()-lx+fz)-(-lx_val+fz_val))*2.0*(-lx_val+fz_val) ) - LinExpr(mVars[mUbVar[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                        lb_y = ( LinExpr(std::pow( lz_val-fx_val, 2.0) + std::pow(-lx_val-fz_val, 2.0)) + (( lz-fx)-( lz_val-fx_val))*2.0*( lz_val-fx_val) + ((LinExpr()-lx-fz)-(-lx_val-fz_val))*2.0*(-lx_val-fz_val) ) - LinExpr(mVars[mLbVar[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                        ub_z = ( LinExpr(std::pow(-ly_val+fx_val, 2.0) + std::pow( lx_val+fy_val, 2.0)) + ((LinExpr()-ly+fx)-(-ly_val+fx_val))*2.0*(-ly_val+fx_val) + (( lx+fy)-( lx_val+fy_val))*2.0*( lx_val+fy_val) ) - LinExpr(mVars[mUbVar[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);
                        lb_z = ( LinExpr(std::pow(-ly_val-fx_val, 2.0) + std::pow( lx_val-fy_val, 2.0)) + ((LinExpr()-ly-fx)-(-ly_val-fx_val))*2.0*(-ly_val-fx_val) + (( lx-fy)-( lx_val-fy_val))*2.0*( lx_val-fy_val) ) - LinExpr(mVars[mLbVar[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]);

                        double w_soft_constraint = mModel.getStgs().get(SolverDoubleParam_SoftConstraintWeight);
                        mQuadObjective.addQuaTerm(w_soft_constraint, ub_x);
                        mQuadObjective.addQuaTerm(w_soft_constraint, lb_x);
                        mQuadObjective.addQuaTerm(w_soft_constraint, ub_y);
                        mQuadObjective.addQuaTerm(w_soft_constraint, lb_y);
                        mQuadObjective.addQuaTerm(w_soft_constraint, ub_z);
                        mQuadObjective.addQuaTerm(w_soft_constraint, lb_z);
                    }
                }
            }
        }

        mModel.setObjective(mQuadObjective, 0.0);

        // constant time horizon with time adaptation
        if (mCentParam->isTimeHorizonFixed && mCentParam->heuristic == Heuristic::timeOptimization) {
            mLinCons = 0.0;
            for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
                mLinCons += mVars[mDt.id(0,time_id)];
            mModel.addLinConstr(mLinCons, "=", mCentParam->numTimeSteps*mCentParam->timeStep);
        }

        // upper and lower bounds constraints
        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) {
            if (mCentParam->heuristic == Heuristic::timeOptimization) {
                mModel.addLinConstr(mVars[mDt.id(0,time_id)], ">", mCentParam->timeRange[0]);
                mModel.addLinConstr(mVars[mDt.id(0,time_id)], "<", mCentParam->timeRange[1]);
            }
            for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                    mModel.addLinConstr(mVars[mTrqLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], ">", mCentParam->torqueRange[0]);
                    mModel.addLinConstr(mVars[mTrqLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], "<", mCentParam->torqueRange[1]);
                    mModel.addLinConstr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], ">", mCentParam->copRange[eef_id][0]);
                    mModel.addLinConstr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], "<", mCentParam->copRange[eef_id][1]);
                    mModel.addLinConstr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], ">", mCentParam->copRange[eef_id][2]);
                    mModel.addLinConstr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], "<", mCentParam->copRange[eef_id][3]);
                }
            }
        }

        // friction cone constraints
        for (int time_id = 0; time_id < mCentParam->numTimeSteps; time_id++) {
            for (int eef_id = 0; eef_id < mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsContactType[eef_id] != ContactType::FullContact) {
                    if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                        Eigen::Matrix3d eff_rotation = mDynStateSeq.dynamicsStateSequence[time_id].eEfsOrientation[eef_id].toRotationMatrix();
                        if (mCentParam->isFrictionConeLinear) {  // using a linear representation
                            Eigen::Matrix<double,4,3> rotated_mConeMatrix = mConeMatrix*eff_rotation.transpose();
                            for (int row_id=0; row_id<4; row_id++) {
                                mLinCons = 0.0;
                                for (int axis_id=0; axis_id<3; axis_id++)
                                    mLinCons += LinExpr(mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rotated_mConeMatrix(row_id,axis_id);
                                mModel.addLinConstr(mLinCons, "<", 0.0);
                            }
                        } else {  // using a second-order cone representation
                            LinExpr fx = 0.0, fy = 0.0, fz = 0.0;
                            for (int axis_id=0; axis_id<3; axis_id++) {
                                fx += LinExpr(mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*eff_rotation(axis_id,0);
                                fy += LinExpr(mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*eff_rotation(axis_id,1);
                                fz += LinExpr(mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*eff_rotation(axis_id,2);
                            }
                            mQuadCons.clear();    mQuadCons.addQuaTerm(1.0, fx);    mQuadCons.addQuaTerm(1.0, fy);
                            mModel.addSocConstr(mQuadCons, "<", fz*mCentParam->frictionCoeff);
                        }
                    }
                }
            }
        }

        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) {
            if (mCentParam->heuristic == Heuristic::timeOptimization) {
                if (is_first_time) {
                    // center of mass constraint
                    for (int axis_id=0; axis_id<3; axis_id++) {
                        if (time_id==0) { mLinCons = LinExpr(mVars[mCom.id(axis_id,time_id)]) - LinExpr(mCentParam->initialState.com[axis_id]) - LinExpr(mVars[mLMom.id(axis_id,time_id)])*(mDynStateSeq.dynamicsStateSequence[time_id].time/mCentParam->robotMass); }
                        else            { mLinCons = LinExpr(mVars[mCom.id(axis_id,time_id)]) - LinExpr(mVars[mCom.id(axis_id,time_id-1)])  - LinExpr(mVars[mLMom.id(axis_id,time_id)])*(mDynStateSeq.dynamicsStateSequence[time_id].time/mCentParam->robotMass); }
                        mModel.addLinConstr(mLinCons, "=", 0.0);
                    }

                    // linear momentum constraint
                    for (int axis_id=0; axis_id<3; axis_id++) {
                        if (time_id==0) { mLinCons = LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mCentParam->initialState.lMom[axis_id]) - LinExpr(mVars[mLMomD.id(axis_id,time_id)])*mDynStateSeq.dynamicsStateSequence[time_id].time; }
                        else            { mLinCons = LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mVars[mLMom.id(axis_id,time_id-1)])   - LinExpr(mVars[mLMomD.id(axis_id,time_id)])*mDynStateSeq.dynamicsStateSequence[time_id].time; }
                        mModel.addLinConstr(mLinCons, "=", 0.0);
                    }

                    // angular momentum constraint
                    for (int axis_id=0; axis_id<3; axis_id++) {
                        if (time_id==0) { mLinCons = LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mCentParam->initialState.aMom[axis_id]) - LinExpr(mVars[mAMomD.id(axis_id,time_id)])*mDynStateSeq.dynamicsStateSequence[time_id].time; }
                        else            { mLinCons = LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mVars[mAMom.id(axis_id,time_id-1)])    - LinExpr(mVars[mAMomD.id(axis_id,time_id)])*mDynStateSeq.dynamicsStateSequence[time_id].time; }
                        mModel.addLinConstr(mLinCons, "=", 0.0);
                    }
                } else {
                    LinExpr lmom, lmomd, amomd, dt = mVars[mDt.id(0,time_id)];
                    double lmom_val, lmomd_val, amomd_val, dt_val = mDynStateSeq.dynamicsStateSequence[time_id].time;

                    // center of mass constraint
                    for (int axis_id=0; axis_id<3; axis_id++) {
                        lmom  = mVars[mLMom.id(axis_id,time_id)];
                        lmom_val  = mDynStateSeq.dynamicsStateSequence[time_id].lMom[axis_id];
                        mLinCons = 0.0;
                        if (time_id==0) { mLinCons += LinExpr(mCentParam->initialState.com[axis_id]) - LinExpr(mVars[mCom.id(axis_id,time_id)]); }
                        else            { mLinCons += LinExpr(mVars[mCom.id(axis_id,time_id-1)]) - LinExpr(mVars[mCom.id(axis_id,time_id)]); }
                        mLinCons += ( LinExpr(std::pow(dt_val+lmom_val, 2.0)) + ((dt+lmom)-(dt_val+lmom_val))*2.0*(dt_val+lmom_val) )*(0.25/mCentParam->robotMass)
                            - ( LinExpr(std::pow(dt_val-lmom_val, 2.0)) + ((dt-lmom)-(dt_val-lmom_val))*2.0*(dt_val-lmom_val) )*(0.25/mCentParam->robotMass);
                        mModel.addLinConstr(mLinCons, "=", 0.0);
                    }

                    // linear momentum constraint
                    for (int axis_id=0; axis_id<3; axis_id++) {
                        lmomd  = mVars[mLMomD.id(axis_id,time_id)];
                        lmomd_val  = mDynStateSeq.dynamicsStateSequence[time_id].lMomD[axis_id];
                        mLinCons = 0.0;
                        if (time_id==0) { mLinCons += LinExpr(mCentParam->initialState.lMom[axis_id]) - LinExpr(mVars[mLMom.id(axis_id,time_id)]); }
                        else            { mLinCons += LinExpr(mVars[mLMom.id(axis_id,time_id-1)]) - LinExpr(mVars[mLMom.id(axis_id,time_id)]); }
                        mLinCons += ( LinExpr(std::pow(dt_val+lmomd_val, 2.0)) + ((dt+lmomd)-(dt_val+lmomd_val))*2.0*(dt_val+lmomd_val) )*0.25
                            - ( LinExpr(std::pow(dt_val-lmomd_val, 2.0)) + ((dt-lmomd)-(dt_val-lmomd_val))*2.0*(dt_val-lmomd_val) )*0.25;
                        mModel.addLinConstr(mLinCons, "=", 0.0);
                    }

                    // angular momentum constraint
                    for (int axis_id=0; axis_id<3; axis_id++) {
                        amomd  = mVars[mAMomD.id(axis_id,time_id)];
                        amomd_val  = mDynStateSeq.dynamicsStateSequence[time_id].aMomD[axis_id];
                        mLinCons = 0.0;
                        if (time_id==0) { mLinCons += LinExpr(mCentParam->initialState.aMom[axis_id]) - LinExpr(mVars[mAMom.id(axis_id,time_id)]); }
                        else            { mLinCons += LinExpr(mVars[mAMom.id(axis_id,time_id-1)]) - LinExpr(mVars[mAMom.id(axis_id,time_id)]); }
                        mLinCons += ( LinExpr(std::pow(dt_val+amomd_val, 2.0)) + ((dt+amomd)-(dt_val+amomd_val))*2.0*(dt_val+amomd_val) )*0.25
                            - ( LinExpr(std::pow(dt_val-amomd_val, 2.0)) + ((dt-amomd)-(dt_val-amomd_val))*2.0*(dt_val-amomd_val) )*0.25;
                        mModel.addLinConstr(mLinCons, "=", 0.0);
                    }
                }

                // linear momentum rate constraint
                for (int axis_id=0; axis_id<3; axis_id++) {
                    mLinCons = LinExpr(mCentParam->robotMass*mCentParam->gravityVector[axis_id]) - LinExpr(mVars[mLMomD.id(axis_id,time_id)]);
                    for (int eef_id=0; eef_id < mCentParam->numActEEfs; eef_id++)
                        if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) { mLinCons += LinExpr(mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*mCentParam->massTimesGravity; }
                    mModel.addLinConstr(mLinCons, "=", 0.0);
                }

                // angular momentum rate constraint
                for (int axis_id=0; axis_id<3; axis_id++) {
                    mLinCons = LinExpr() - LinExpr(mVars[mAMomD.id(axis_id,time_id)]);
                    for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
                        if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                            Eigen::Matrix3d rot = mDynStateSeq.dynamicsStateSequence[time_id].eEfsOrientation[eef_id].toRotationMatrix();
                            mLinCons += (LinExpr(mVars[mUbVar[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])-LinExpr(mVars[mLbVar[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]))*0.25*mCentParam->massTimesGravity;
                            mLinCons += LinExpr(mVars[mTrqLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rot.col(2)[axis_id];
                        }
                    }
                    mModel.addLinConstr(mLinCons, "=", 0.0);
                }
            } else {
                // center of mass constraint
                for (int axis_id=0; axis_id<3; axis_id++) {
                    if (time_id==0) {
                        mLinCons = LinExpr(mVars[mCom.id(axis_id,time_id)]) - LinExpr(mCentParam->initialState.com[axis_id]) - LinExpr(mVars[mLMom.id(axis_id,time_id)])*(mDynStateSeq.dynamicsStateSequence[time_id].time/mCentParam->robotMass);
                    }
                    else {
                        mLinCons = LinExpr(mVars[mCom.id(axis_id,time_id)]) - LinExpr(mVars[mCom.id(axis_id,time_id-1)])  - LinExpr(mVars[mLMom.id(axis_id,time_id)])*(mDynStateSeq.dynamicsStateSequence[time_id].time/mCentParam->robotMass);
                    }
                    mModel.addLinConstr(mLinCons, "=", 0.0);
                }

                // linear momentum constraint
                for (int axis_id=0; axis_id<3; axis_id++) {
                    if (time_id==0) {
                        mLinCons = LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].time*(mCentParam->robotMass*mCentParam->gravityVector[axis_id])) - LinExpr(mCentParam->initialState.lMom(axis_id));
                    }
                    else {
                        mLinCons = LinExpr(mVars[mLMom.id(axis_id,time_id)]) - LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].time*(mCentParam->robotMass*mCentParam->gravityVector[axis_id])) - LinExpr(mVars[mLMom.id(axis_id,time_id-1)]);
                    }

                    for (int eef_id=0; eef_id < mCentParam->numActEEfs; eef_id++) {
                        if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                            mLinCons += LinExpr(mVars[mFrcWorld[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*(-mDynStateSeq.dynamicsStateSequence[time_id].time*mCentParam->massTimesGravity);
                        }
                    }
                    if (time_id==0) { mLinCons += LinExpr(mCentParam->externalForce[axis_id])*(-mDynStateSeq.dynamicsStateSequence[time_id].time*mCentParam->massTimesGravity); }
                    mModel.addLinConstr(mLinCons, "=", 0.0);
                }

                // angular momentum constraint
                for (int axis_id=0; axis_id<3; axis_id++) {
                    if (time_id==0) {
                        mLinCons = LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mCentParam->initialState.aMom[axis_id]);
                    }
                    else {
                        mLinCons = LinExpr(mVars[mAMom.id(axis_id,time_id)]) - LinExpr(mVars[mAMom.id(axis_id,time_id-1)]);
                    }

                    for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
                        if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                            mLinCons += (LinExpr(mVars[mLbVar[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])-LinExpr(mVars[mUbVar[eef_id].id(axis_id,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])]))*(0.25*mDynStateSeq.dynamicsStateSequence[time_id].time*mCentParam->massTimesGravity);

                            Eigen::Vector3d rz = mDynStateSeq.dynamicsStateSequence[time_id].eEfsOrientation[eef_id].toRotationMatrix().col(2);
                            mLinCons += LinExpr(mVars[mTrqLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*(-rz[axis_id]*mDynStateSeq.dynamicsStateSequence[time_id].time);
                        }
                    }
                    mModel.addLinConstr(mLinCons, "=", 0.0);
                }
            }
        }

        QuadConstrApprox qapprox = QuadConstrApprox::None;
        switch (mCentParam->heuristic) {
            case Heuristic::timeOptimization: { qapprox = QuadConstrApprox::None; break; }
            case Heuristic::trustRegion: { qapprox = QuadConstrApprox::TrustRegion; break; }
            case Heuristic::softConstraint: { qapprox = QuadConstrApprox::SoftConstraint; break; }
            default: { qapprox = QuadConstrApprox::None; break; }
        }

        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) {
            for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                    Eigen::Matrix3d rot = mDynStateSeq.dynamicsStateSequence[time_id].eEfsOrientation[eef_id].toRotationMatrix();
                    Eigen::Vector3d rx = rot.col(0);
                    Eigen::Vector3d ry = rot.col(1);

                    LinExpr lx, ly , lz, fx, fy, fz;
                    lx = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].x()) - LinExpr(mVars[mCom.id(0,time_id)]) + LinExpr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rx(0) + LinExpr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*ry(0);
                    ly = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].y()) - LinExpr(mVars[mCom.id(1,time_id)]) + LinExpr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rx(1) + LinExpr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*ry(1);
                    lz = LinExpr(mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].z()) - LinExpr(mVars[mCom.id(2,time_id)]) + LinExpr(mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*rx(2) + LinExpr(mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])])*ry(2);
                    fx = mVars[mFrcWorld[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])];
                    fy = mVars[mFrcWorld[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])];
                    fz = mVars[mFrcWorld[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])];

                    // upper and lower bounds on momentum rate constraints
                    mQuadCons.clear();    mQuadCons.addQuaTerm(1.0, LinExpr()-lz+fy);    mQuadCons.addQuaTerm(1.0,  ly+fz);
                    mModel.addQuaConstr(mQuadCons, "<", mVars[mUbVar[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], qapprox);

                    mQuadCons.clear();    mQuadCons.addQuaTerm(1.0,  lz+fx);    mQuadCons.addQuaTerm(1.0, LinExpr()-lx+fz);
                    mModel.addQuaConstr(mQuadCons, "<", mVars[mUbVar[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], qapprox);

                    mQuadCons.clear();    mQuadCons.addQuaTerm(1.0, LinExpr()-ly+fx);    mQuadCons.addQuaTerm(1.0,  lx+fy);
                    mModel.addQuaConstr(mQuadCons, "<", mVars[mUbVar[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], qapprox);

                    mQuadCons.clear();    mQuadCons.addQuaTerm(1.0, LinExpr()-lz-fy);    mQuadCons.addQuaTerm(1.0,  ly-fz);
                    mModel.addQuaConstr(mQuadCons, "<", mVars[mLbVar[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], qapprox);

                    mQuadCons.clear();    mQuadCons.addQuaTerm(1.0,  lz-fx);    mQuadCons.addQuaTerm(1.0, LinExpr()-lx-fz);
                    mModel.addQuaConstr(mQuadCons, "<", mVars[mLbVar[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], qapprox);

                    mQuadCons.clear();    mQuadCons.addQuaTerm(1.0, LinExpr()-ly-fx);    mQuadCons.addQuaTerm(1.0,  lx-fy);
                    mModel.addQuaConstr(mQuadCons, "<", mVars[mLbVar[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])], qapprox);

                    // end-effector length constraint
                    mQuadCons.clear();  mQuadCons.addQuaTerm(1.0, lx-mCentParam->eEfOffset[eef_id].x());  mQuadCons.addQuaTerm(1.0, ly-mCentParam->eEfOffset[eef_id].y());  mQuadCons.addQuaTerm(1.0, lz-mCentParam->eEfOffset[eef_id].z());
                    mModel.addQuaConstr(mQuadCons, "<", pow(mCentParam->maxEEfLengths[eef_id],2.0));
                }
            }
        }

        // formulate problem in standard conic form and solve it
        mTimer.start();
        mExitCode = mModel.optimize();
        mSolveTime += mTimer.stop();

        // extract solution
        mSolution.resize(mNumVars, 1);  mSolution.setZero();
        for (int var_id=0; var_id<mNumVars; var_id++) { mSolution(var_id) = mVars[var_id].get(SolverDoubleParam_X); }
    }
    catch(...)
    {
        std::cout << "Exception during optimization" << std::endl;
    }

    if (mCentParam->heuristic == Heuristic::timeOptimization)
        _saveSolution(mDt);
    _saveSolution(mCom);
    _saveSolution(mAMom);
    _saveSolution(mLMom);
    if (mCentParam->heuristic == Heuristic::timeOptimization) {
        _saveSolution(mLMomD);
        _saveSolution(mAMomD);
    }
    for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
        _saveSolution(mFrcWorld[eef_id]);
        _saveSolution(mCopLocal[eef_id]);
        _saveSolution(mTrqLocal[eef_id]);
    }

    // computation of linear momentum out of forces
    Eigen::Vector3d frc, len, trq;
    Eigen::MatrixXd compos(3,mCentParam->numTimeSteps); compos.setZero();
    Eigen::MatrixXd linmom(3,mCentParam->numTimeSteps); linmom.setZero();
    Eigen::MatrixXd angmom(3,mCentParam->numTimeSteps); angmom.setZero();

    if (mCentParam->heuristic == Heuristic::timeOptimization) {
        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) {
            if (time_id==0) {
                compos.col(time_id) = mCentParam->initialState.com;
                linmom.col(time_id) = mCentParam->initialState.lMom;
            } else {
                compos.col(time_id) = compos.col(time_id-1);
                linmom.col(time_id) = linmom.col(time_id-1);
            }

            linmom.col(time_id) += mCentParam->robotMass * mCentParam->gravityVector*mVars[mDt.id(0,time_id)].get(SolverDoubleParam_X);
            for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
                if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                    frc.x() = mCentParam->massTimesGravity*mVars[mFrcWorld[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                    frc.y() = mCentParam->massTimesGravity*mVars[mFrcWorld[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                    frc.z() = mCentParam->massTimesGravity*mVars[mFrcWorld[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                    linmom.col(time_id).head(3) += mVars[mDt.id(0,time_id)].get(SolverDoubleParam_X)*frc;
                }
            }
            compos.col(time_id).head(3) += 1.0/mCentParam->robotMass*mVars[mDt.id(0,time_id)].get(SolverDoubleParam_X)*linmom.col(time_id).head(3);
        }
    }

    // computation of angular momentum out of forces and lengths
    for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) {
        if (time_id == 0) { angmom.col(time_id) = mCentParam->initialState.aMom; }
        else              { angmom.col(time_id) = angmom.col(time_id-1); }

        for (int eef_id=0; eef_id < mCentParam->numActEEfs; eef_id++) {
            if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id]) {
                Eigen::Matrix3d rot = mDynStateSeq.dynamicsStateSequence[time_id].eEfsOrientation[eef_id].toRotationMatrix();
                Eigen::Vector3d rx = rot.col(0);
                Eigen::Vector3d ry = rot.col(1);
                Eigen::Vector3d rz = rot.col(2);

                if (mCentParam->heuristic == Heuristic::timeOptimization) {
                    len.x() = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].x() - compos(0,time_id) + rx(0)*mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X) + ry(0)*mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                    len.y() = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].y() - compos(1,time_id) + rx(1)*mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X) + ry(1)*mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                    len.z() = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].z() - compos(2,time_id) + rx(2)*mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X) + ry(2)*mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                } else {
                    len.x() = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].x() - mVars[mCom.id(0,time_id)].get(SolverDoubleParam_X) + rx(0)*mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X) + ry(0)*mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                    len.y() = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].y() - mVars[mCom.id(1,time_id)].get(SolverDoubleParam_X) + rx(1)*mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X) + ry(1)*mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                    len.z() = mDynStateSeq.dynamicsStateSequence[time_id].eEfsPosition[eef_id].z() - mVars[mCom.id(2,time_id)].get(SolverDoubleParam_X) + rx(2)*mVars[mCopLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X) + ry(2)*mVars[mCopLocal[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                }
                frc.x() = mCentParam->massTimesGravity*mVars[mFrcWorld[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                frc.y() = mCentParam->massTimesGravity*mVars[mFrcWorld[eef_id].id(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                frc.z() = mCentParam->massTimesGravity*mVars[mFrcWorld[eef_id].id(2,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                trq     = rz * mVars[mTrqLocal[eef_id].id(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id])].get(SolverDoubleParam_X);
                angmom.col(time_id).head(3) += mDynStateSeq.dynamicsStateSequence[time_id].time*(len.cross(frc)+trq);
            }
        }
    }
    if (mCentParam->heuristic != Heuristic::timeOptimization) { mAMom.setGuessValue(angmom); }
    _storeSolution();

    if (mCentParam->heuristic == Heuristic::timeOptimization) {
        // computing convergence error
        if (is_first_time) { mLastConvergenceErr = SolverSetting::inf; }
        else               { mLastConvergenceErr = mConvergenceErr; }
        mCom.getGuessValue(mComGuess);   double com_err  = (mComGuess-compos).norm()/mCentParam->numTimeSteps;
        mLMom.getGuessValue(mLMomGuess); double lmom_err = (mLMomGuess-linmom).norm()/mCentParam->numTimeSteps;
        mAMom.getGuessValue(mAMomGuess); double amom_err = (mAMomGuess-angmom).norm()/mCentParam->numTimeSteps;
        mConvergenceErr = std::max(com_err, std::max(lmom_err, amom_err));

        if (mConvergenceErr < mCentParam->maxTimeResidualTolerance) { mHasConverged = true; }
        if (std::abs(mLastConvergenceErr-mConvergenceErr) < mCentParam->minTimeResidualImprovement) { mHasConverged = true; }
        if (mHasConverged) { mAMom.setGuessValue(angmom); _storeSolution(); }
    }
}

void CentroidPlanner::_saveToFile(const DynamicsStateSequence& _ref_sequence) {
    if (mCentParam->isStoreData) {
        try
        {
            YAML::Node cfg_pars = YAML::LoadFile(mCentParam->cfgFile.c_str());

            YAML::Node qcqp_cfg;
            qcqp_cfg["robot_model_path"] = cfg_pars["robot_model_path"];
            qcqp_cfg["dynopt_params"]["time_step"] = mCentParam->timeStep;
            qcqp_cfg["dynopt_params"]["end_com"] = mComPosGoal;
            qcqp_cfg["dynopt_params"]["robot_mass"] = mCentParam->robotMass;
            qcqp_cfg["dynopt_params"]["n_act_eefs"] = mCentParam->numActEEfs;
            qcqp_cfg["dynopt_params"]["ini_com"] = mCentParam->initialState.com;
            qcqp_cfg["dynopt_params"]["time_horizon"] = mCentParam->timeHorizon;
            qcqp_cfg["cntopt_params"] = cfg_pars["contact_plan"];

            mCom.getGuessValue(mMatGuess);   qcqp_cfg["dynopt_params"]["com_motion"] = mMatGuess;
            mLMom.getGuessValue(mMatGuess);  qcqp_cfg["dynopt_params"]["lin_mom"] = mMatGuess;
            mAMom.getGuessValue(mMatGuess);  qcqp_cfg["dynopt_params"]["ang_mom"] = mMatGuess;

            // building momentum references
            for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) { mMatGuess.col(time_id) = _ref_sequence.dynamicsStateSequence[time_id].com;  } qcqp_cfg["dynopt_params"]["com_motion_ref"] = mMatGuess;
            for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) { mMatGuess.col(time_id) = _ref_sequence.dynamicsStateSequence[time_id].lMom; } qcqp_cfg["dynopt_params"]["lin_mom_ref"] = mMatGuess;
            for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++) { mMatGuess.col(time_id) = _ref_sequence.dynamicsStateSequence[time_id].aMom; } qcqp_cfg["dynopt_params"]["ang_mom_ref"] = mMatGuess;

            // saving vector of time-steps
            mMatGuess.resize(1, mCentParam->numTimeSteps); mMatGuess.setZero();
            mMatGuess(0,0) = mDynStateSeq.dynamicsStateSequence[0].time;
            for (int time_id=1; time_id<mCentParam->numTimeSteps; time_id++)
                mMatGuess(0,time_id) = mMatGuess(0,time_id-1) + mDynStateSeq.dynamicsStateSequence[time_id].time;
            qcqp_cfg["dynopt_params"]["time_vec"] = mMatGuess;

            // saving vector of forces, torques and cops
            for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
                mMatGuess.resize(3, mCentParam->numTimeSteps); mMatGuess.setZero();
                for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
                    if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id])
                        mMatGuess.col(time_id).head(3) = mDynStateSeq.dynamicsStateSequence[time_id].eEfsFrc[eef_id];
                qcqp_cfg["dynopt_params"]["eef_frc_"+std::to_string(eef_id)] = mMatGuess;

                mMatGuess.resize(1, mCentParam->numTimeSteps); mMatGuess.setZero();
                for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
                    if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id])
                        mMatGuess(0, time_id) = mDynStateSeq.dynamicsStateSequence[time_id].eEfsTrq[eef_id].z();
                qcqp_cfg["dynopt_params"]["eef_trq_"+std::to_string(eef_id)] = mMatGuess;

                mCopLocal[eef_id].getGuessValue(mMatGuess);
                qcqp_cfg["dynopt_params"]["eef_cop_"+std::to_string(eef_id)] = mMatGuess;
            }
            std::ofstream file_out(mCentParam->saveDynamicsFile); file_out << qcqp_cfg;
        }
        catch (YAML::ParserException &e) { std::cout << e.what() << "\n"; }
    }
}

void CentroidPlanner::_saveSolution(solver::OptimizationVariable& opt_var) {
    mMatGuess.resize(opt_var.getNumRows(), opt_var.getNumCols()); mMatGuess.setZero();
    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++) {
        for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
            mMatGuess(row_id,col_id) = mSolution(opt_var.id(row_id,col_id));
    }
    opt_var.setGuessValue(mMatGuess);
}

void CentroidPlanner::_storeSolution() {
    mCom.getGuessValue(mMatGuess);
    for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].com = Eigen::Vector3d(mMatGuess.block<3,1>(0,time_id));

    mLMom.getGuessValue(mMatGuess);
    for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].lMom = Eigen::Vector3d(mMatGuess.block<3,1>(0,time_id));

    mAMom.getGuessValue(mMatGuess);
    for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
        mDynStateSeq.dynamicsStateSequence[time_id].aMom = Eigen::Vector3d(mMatGuess.block<3,1>(0,time_id));

    if (mCentParam->heuristic == Heuristic::timeOptimization) {
        mDt.getGuessValue(mMatGuess);
        for (int time=0; time<mCentParam->numTimeSteps; time++)
            mDynStateSeq.dynamicsStateSequence[time].time = mMatGuess(0,time);

        mLMomD.getGuessValue(mMatGuess);
        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
            mDynStateSeq.dynamicsStateSequence[time_id].lMomD = Eigen::Vector3d(mMatGuess.block<3,1>(0,time_id));

        mAMomD.getGuessValue(mMatGuess);
        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
            mDynStateSeq.dynamicsStateSequence[time_id].aMomD = Eigen::Vector3d(mMatGuess.block<3,1>(0,time_id));
    }

    for (int eef_id=0; eef_id<mCentParam->numActEEfs; eef_id++) {
        mFrcWorld[eef_id].getGuessValue(mMatGuess);
        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
            if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id])
                mDynStateSeq.dynamicsStateSequence[time_id].eEfsFrc[eef_id] = Eigen::Vector3d(mMatGuess.block<3,1>(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id]));

        mCopLocal[eef_id].getGuessValue(mMatGuess);
        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
            if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id])
                mDynStateSeq.dynamicsStateSequence[time_id].eEfsCop[eef_id] = Eigen::Vector3d(mMatGuess(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id]), mMatGuess(1,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id]), 0.0);

        mTrqLocal[eef_id].getGuessValue(mMatGuess);
        for (int time_id=0; time_id<mCentParam->numTimeSteps; time_id++)
            if (mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivation[eef_id])
                mDynStateSeq.dynamicsStateSequence[time_id].eEfsTrq[eef_id] = Eigen::Vector3d(0.0, 0.0, mMatGuess(0,mDynStateSeq.dynamicsStateSequence[time_id].eEfsActivationIds[eef_id]));
    }
}

void CentroidPlanner::_addVariableToModel(const OptimizationVariable& opt_var, Model& model, std::vector<Var>& vars)
{
    opt_var.getValues(mMatLb, mMatUb, mMatGuess, mSize, mVariableType);
    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
        for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
            switch (mVariableType) {
                case 'C': { vars[opt_var.id(row_id,col_id)] = model.addVar(VarType::Continuous, double(mMatLb(row_id,col_id)), double(mMatUb(row_id,col_id)), double(mMatGuess(row_id,col_id))); break; }
                default: { throw std::runtime_error("At add_var_to_model, variable type not handled"); }
            }

    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
        for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
            vars[opt_var.id(row_id,col_id)].set(SolverDoubleParam_X, mMatGuess(row_id,col_id));
}
