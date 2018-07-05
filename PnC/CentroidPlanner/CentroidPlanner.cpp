#include "CentroidPlanner.hpp"
#include "Utilities.hpp"

using namespace myUtils;

void CentroidPlannerParameter::paramSetFromYaml(const std::string & cfg_file) {
    cfgFile = cfg_file;
    try {
        YAML::Node planner_cfg = YAML::LoadFile(cfgFile.c_str());

        // ===============
        // Planner Setting
        // ===============
        YAML::Node planner_vars = planner_cfg["planner_variable"];
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
        for (int eff_id=0; eff_id<CentroidModel::numEEf; eff_id++) {
            readParameter(planner_vars, "cop_range_"+CentroidModel::eEfIdToString(eff_id), copRange[eff_id]);
            readParameter(planner_vars, "eef_offset_"+CentroidModel::eEfIdToString(eff_id), eEfOffset[eff_id]);
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

        // ===================
        // Robot Initial State
        // ===================
        YAML::Node ini_robo_cfg = planner_cfg["initial_robot_configuration"];
        readParameter(ini_robo_cfg, "com", initialState.com);
        readParameter(ini_robo_cfg, "lmom", initialState.lMom);
        readParameter(ini_robo_cfg, "amom", initialState.aMom);
        for (int eff_id=0; eff_id<CentroidModel::numEEf; eff_id++) {
            Eigen::VectorXd eef_cfg = readParameter<Eigen::VectorXd>(ini_robo_cfg["eef_pose"], "eef_"+CentroidModel::eEfIdToString(eff_id));
            initialState.eEfsActivation[eff_id] = int(eef_cfg(0));
            initialState.eEfsPosition[eff_id] = eef_cfg.segment<3>(1);
            initialState.eEfsOrientation[eff_id] = Eigen::Quaternion<double>(eef_cfg[4],eef_cfg[5],eef_cfg[6],eef_cfg[7]);
            readParameter(ini_robo_cfg["eef_ctrl"], "eef_frc_"+CentroidModel::eEfIdToString(eff_id), initialState.eEfsFrc[eff_id]);
        }

        // ================
        // Contact Interface
        // ================
        YAML::Node contact_vars = planner_cfg["contact_plan"];

        // Contact parameters
        readParameter(contact_vars, "num_contacts", contactPlanInterface.contactsPerEndeff);
        for (int eff_id=0; eff_id<CentroidModel::numEEf; eff_id++) {
            contactPlanInterface.contactSequence.eEfContacts[eff_id].clear();
            if (contactPlanInterface.contactsPerEndeff[eff_id]>0) {
                YAML::Node eff_params = contact_vars[("effcnt_" + CentroidModel::eEfIdToString(eff_id)).c_str()];
                for (int cnt_id=0; cnt_id<contactPlanInterface.contactsPerEndeff[eff_id]; cnt_id++) {
                    Eigen::VectorXd v(10);
                    readParameter(eff_params, "cnt"+std::to_string(cnt_id), v);
                    contactPlanInterface.contactSequence.eEfContacts[eff_id].push_back(ContactState());
                    contactPlanInterface.contactSequence.eEfContacts[eff_id][cnt_id].timeIni = v[0];
                    contactPlanInterface.contactSequence.eEfContacts[eff_id][cnt_id].timeEnd = v[1];
                    contactPlanInterface.contactSequence.eEfContacts[eff_id][cnt_id].position = v.segment<3>(2);
                    contactPlanInterface.contactSequence.eEfContacts[eff_id][cnt_id].contactType = idToContactType(v(9));
                    contactPlanInterface.contactSequence.eEfContacts[eff_id][cnt_id].orientation = Eigen::Quaternion<double>(v[5],v[6],v[7],v[8]);
                }
            }
        }

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }

}


CentroidPlanner::CentroidPlanner() {}
CentroidPlanner::~CentroidPlanner() {}
void CentroidPlanner::_doPlan() {}
void CentroidPlanner::_evalTrajectory(double time,
        Eigen::VectorXd & pos,
        Eigen::VectorXd & vel,
        Eigen::VectorXd &trq) {

}
