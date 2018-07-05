#pragma once

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <ContactState.hpp>
#include <DynamicsState.hpp>
#include "CentroidModel.hpp"

class ContactPlanInterface
{
public:
    ContactPlanInterface (){};
    virtual ~ContactPlanInterface (){};

    void fillDynamicsSequence(DynamicsStateSequence& dynamics_sequence,
                              int numTimeSteps,
                              double timeStep) {
        // configuration of end-effectors during contact
        dynamics_sequence.activeEEfSteps.setZero();
        for (int eff_id=0; eff_id<CentroidModel::numEEf; eff_id++) {
            int counter = 0;
            int ini_id = 0, end_id = contactSequence.eEfContacts[eff_id].size();
            for (int cnt_id=ini_id; cnt_id<end_id; cnt_id++) {
                for (int time_id=0; time_id<numTimeSteps; time_id++) {
                    double current_time = double(time_id+1.0)*timeStep;
                    if (current_time>=contactSequence.eEfContacts[eff_id][cnt_id].timeIni &&
                            current_time< contactSequence.eEfContacts[eff_id][cnt_id].timeEnd)
                    {
                        dynamics_sequence.dynamicsStateSequence[time_id].eEfsActivation[eff_id] = true;
                        dynamics_sequence.dynamicsStateSequence[time_id].eEfsIds[eff_id] = counter++;
                        dynamics_sequence.dynamicsStateSequence[time_id].eEfsContactType[eff_id] = contactSequence.eEfContacts[eff_id][cnt_id].contactType;
                        dynamics_sequence.dynamicsStateSequence[time_id].eEfsPosition[eff_id] = contactSequence.eEfContacts[eff_id][cnt_id].position;
                        dynamics_sequence.dynamicsStateSequence[time_id].eEfsOrientation[eff_id] = contactSequence.eEfContacts[eff_id][cnt_id].orientation;
                    }
                }
            }
            dynamics_sequence.activeEEfSteps[eff_id] = counter;
        }
    }

    ContactStateSequence contactSequence;
    Eigen::Matrix<int, CentroidModel::numEEf, 1> contactsPerEndeff;
};
