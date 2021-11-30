#pragma once

#include <Eigen/Eigen>
#include <pnc/planners/locomotion/centroid_planner/contact_state.hpp>
#include <pnc/robot_system/centroid_model.hpp>
#include <array>
#include <iostream>
#include <string>
#include <vector>

class DynamicsState {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::array<int, CentroidModel::numEEf> IntArray;
    typedef std::array<bool, CentroidModel::numEEf> BoolArray;
    typedef std::array<ContactType, CentroidModel::numEEf> CntTypeArray;
    typedef std::array<Eigen::Vector3d, CentroidModel::numEEf> Vec3dArray;
    typedef std::array<Eigen::Quaternion<double>, CentroidModel::numEEf>
        OriArray;

    DynamicsState()
        : time(0.),
          com(Eigen::Vector3d::Zero()),
          aMom(Eigen::Vector3d::Zero()),
          lMom(Eigen::Vector3d::Zero()),
          aMomD(Eigen::Vector3d::Zero()),
          lMomD(Eigen::Vector3d::Zero()) {
        for (int eff = 0; eff < CentroidModel::numEEf; ++eff) {
            eEfsActivationIds[eff] = 0;
            eEfsFrc[eff] = Eigen::Vector3d::Zero();
            eEfsTrq[eff] = Eigen::Vector3d::Zero();
            eEfsCop[eff] = Eigen::Vector3d::Zero();

            eEfsActivation[eff] = false;
            eEfsPosition[eff] = Eigen::Vector3d::Zero();
            eEfsContactType[eff] = ContactType::FreeContact;
            eEfsOrientation[eff] = Eigen::Quaternion<double>::Identity();
        }
    };

    virtual ~DynamicsState(){};

    std::string toString() const {
        std::stringstream text;
        text << "  com     " << com.transpose() << "\n";
        text << "  lmom    " << lMom.transpose() << "\n";
        text << "  amom    " << aMom.transpose() << "\n";

        for (int eff = 0; eff < CentroidModel::numEEf; eff++) {
            text << "  eff   " << eff << "\n";
            text << "  eff activation id  " << eEfsActivationIds[eff] << "\n";
            text << "  act " << eEfsActivation[eff] << "\n";
            text << "  pos " << eEfsPosition[eff].transpose() << "\n";
            text << "  ori " << (eEfsOrientation[eff]).coeffs().transpose()
                 << "\n";

            text << "  cop " << (eEfsCop[eff]).transpose() << "\n";
            text << "  frc " << (eEfsFrc[eff]).transpose() << "\n";
            text << "  trq " << (eEfsTrq[eff]).transpose() << "\n";
        }
        return text.str();
    };
    friend std::ostream& operator<<(std::ostream& os,
                                    const DynamicsState& obj) {
        return os << obj.toString();
    }

    double time;
    Eigen::Vector3d com, aMom, lMom, aMomD, lMomD;

    IntArray eEfsActivationIds;
    BoolArray eEfsActivation;
    OriArray eEfsOrientation;
    CntTypeArray eEfsContactType;
    Vec3dArray eEfsFrc, eEfsTrq, eEfsCop, eEfsPosition;
};

class DynamicsStateSequence {
   public:
    DynamicsStateSequence(){};
    virtual ~DynamicsStateSequence(){};

    void resize(int num_timesteps) {
        dynamicsStateSequence.clear();
        for (int i = 0; i < num_timesteps; ++i)
            dynamicsStateSequence.push_back(DynamicsState());
    };
    void clean() { dynamicsStateSequence.clear(); }
    int size() const { return dynamicsStateSequence.size(); }
    std::string toString() const {
        std::stringstream text;
        for (int time_id = 0; time_id < this->size(); time_id++) {
            text << "time " << time_id << "\n";
            text << dynamicsStateSequence[time_id];
        }
        return text.str();
    };
    friend std::ostream& operator<<(std::ostream& os,
                                    const DynamicsStateSequence& obj) {
        return os << obj.toString();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<DynamicsState> dynamicsStateSequence;
    Eigen::Matrix<int, CentroidModel::numEEf, 1> activeEEfSteps;
};
