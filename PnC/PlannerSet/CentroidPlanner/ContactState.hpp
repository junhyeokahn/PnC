#pragma once

#include <Eigen/Eigen>
#include <PnC/RobotSystem/CentroidModel.hpp>
#include <array>
#include <iostream>
#include <memory>
#include <vector>

enum class ContactType { FreeContact = 0, FlatContact = 1, FullContact = 2 };
inline ContactType idToContactType(int cnt_type_id) {
    ContactType cnt_type = ContactType::FreeContact;
    switch (cnt_type_id) {
        case 0: {
            cnt_type = ContactType::FreeContact;
            break;
        }
        case 1: {
            cnt_type = ContactType::FlatContact;
            break;
        }
        case 2: {
            cnt_type = ContactType::FullContact;
            break;
        }
    }
    return cnt_type;
}

class ContactState {
   public:
    ContactState()
        : timeIni(0.),
          timeEnd(0.),
          contactType(ContactType::FreeContact),
          position(Eigen::Vector3d::Zero()),
          orientation(Eigen::Quaternion<double>::Identity()){

          };
    virtual ~ContactState(){};

    std::string toString() const {
        std::stringstream text;
        text << "    time         " << timeIni << " - " << timeEnd << "\n";
        text << "    contact type " << static_cast<int>(contactType) << "\n";
        text << "    position     " << position.transpose() << "\n";
        text << "    orientation  " << orientation.coeffs().transpose() << "\n";
        return text.str();
    };
    friend std::ostream& operator<<(std::ostream& os, const ContactState& obj) {
        return os << obj.toString();
    }

    Eigen::Vector3d position;
    ContactType contactType;
    double timeIni, timeEnd;
    Eigen::Quaternion<double> orientation;
};

class ContactStateSequence {
   public:
    ContactStateSequence(){};
    virtual ~ContactStateSequence(){};

    std::string toString() const {
        std::stringstream text;
        for (int eff_id = 0; eff_id < CentroidModel::numEEf; eff_id++) {
            text << "eff_id " << eff_id << "\n";
            for (int cnt_id = 0; cnt_id < eEfContacts[eff_id].size();
                 cnt_id++) {
                text << "  cnt_id " << cnt_id << "\n";
                text << eEfContacts[eff_id][cnt_id];
            }
        }
        return text.str();
    };
    friend std::ostream& operator<<(std::ostream& os,
                                    const ContactStateSequence& obj) {
        return os << obj.toString();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::array<std::vector<ContactState>, CentroidModel::numEEf> eEfContacts;
};
