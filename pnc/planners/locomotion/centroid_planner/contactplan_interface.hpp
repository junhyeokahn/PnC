#pragma once

#include <Eigen/Eigen>
#include <pnc/planners/locomotion/centroid_planner/contact_state.hpp>
#include <pnc/planners/locomotion/centroid_planner/dynamics_state.hpp>
#include <pnc/robot_system/centroid_model.hpp>
#include <array>
#include <iostream>
#include <string>
#include <vector>

class ContactPlanInterface {
   public:
    ContactPlanInterface(){};
    virtual ~ContactPlanInterface(){};

    void fillDynamicsSequence(DynamicsStateSequence& dynamics_sequence,
                              int numTimeSteps, double timeStep) {
        // configuration of end-effectors during contact
        dynamics_sequence.activeEEfSteps.setZero();
        for (int eef_id = 0; eef_id < CentroidModel::numEEf; eef_id++) {
            int counter = 0;
            int ini_id = 0, end_id = contactSequence.eEfContacts[eef_id].size();
            for (int cnt_id = ini_id; cnt_id < end_id; cnt_id++) {
                for (int time_id = 0; time_id < numTimeSteps; time_id++) {
                    double current_time = double(time_id + 1.0) * timeStep;
                    if (current_time >=
                            contactSequence.eEfContacts[eef_id][cnt_id]
                                .timeIni &&
                        current_time <
                            contactSequence.eEfContacts[eef_id][cnt_id]
                                .timeEnd) {
                        dynamics_sequence.dynamicsStateSequence[time_id]
                            .eEfsActivation[eef_id] = true;
                        dynamics_sequence.dynamicsStateSequence[time_id]
                            .eEfsActivationIds[eef_id] = counter++;
                        dynamics_sequence.dynamicsStateSequence[time_id]
                            .eEfsContactType[eef_id] =
                            contactSequence.eEfContacts[eef_id][cnt_id]
                                .contactType;
                        dynamics_sequence.dynamicsStateSequence[time_id]
                            .eEfsPosition[eef_id] =
                            contactSequence.eEfContacts[eef_id][cnt_id]
                                .position;
                        dynamics_sequence.dynamicsStateSequence[time_id]
                            .eEfsOrientation[eef_id] =
                            contactSequence.eEfContacts[eef_id][cnt_id]
                                .orientation;
                    }
                }
            }
            dynamics_sequence.activeEEfSteps[eef_id] = counter;
        }
    }

    ContactStateSequence contactSequence;
    Eigen::Matrix<int, CentroidModel::numEEf, 1> contactsPerEndeff;
};
