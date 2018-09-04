#include "InvKinPlanner.hpp"
#include "Utilities.hpp"

using namespace myUtils;

void InvKinPlannerParameter::paramSetFromYaml(const std::string & cfg_file) {
    cfgFile = cfg_file;
    saveSolutionFiles = cfg_file.substr(0, cfg_file.size()-12) + "_KINEMATIC_RESULT.yaml";
    try {
        Eigen::MatrixXd dummyMat;
        YAML::Node planner_cfg = YAML::LoadFile(cfgFile.c_str());
        readParameter(planner_cfg, "robot_model_path", robotModelPath);
        robot = std::make_shared<RobotSystem>(6, THIS_COM+robotModelPath+"Draco.urdf");

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

        YAML::Node cntopt_params = planner_cfg["cntopt_params"];
        readParameter(cntopt_params, "num_contacts", contactPlanInterface.contactsPerEndeff);
        for (int eef_id=0; eef_id<CentroidModel::numEEf; eef_id++) {
            contactPlanInterface.contactSequence.eEfContacts[eef_id].clear();
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
                }
            }
        }

        // TODO : make endeffector desired traj
        exit(0);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
}

InvKinPlanner::InvKinPlanner() : Planner() {}
InvKinPlanner::~InvKinPlanner() {}

void InvKinPlanner::_doPlan() {
    mInvKinParam = std::dynamic_pointer_cast<InvKinPlannerParameter>(mParam);
}

void InvKinPlanner::_evalTrajectory( double time,
                                     Eigen::VectorXd & pos,
                                     Eigen::VectorXd & vel,
                                     Eigen::VectorXd & trq ) {

}
