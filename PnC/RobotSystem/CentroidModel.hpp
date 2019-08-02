#pragma once

#include <stdio.h>
#include <stdexcept>
#include <string>

class CentroidModel {
   public:
    // Number of EndEffector
    static constexpr int numEEf = 4;

    // ID of EndEffector
    enum class EEfID {
        rightFoot = 0,
        leftFoot = 1,
        rightHand = 2,
        leftHand = 3
    };

    // Function to differentiate between hands and feet
    static bool isHand(int eefId_) {
        if (eefId_ == static_cast<int>(EEfID::leftHand) ||
            eefId_ == static_cast<int>(EEfID::rightHand)) {
            return true;
        }
        return false;
    }

    // Function to map EndEffector Id to string
    static std::string eEfIdToString(int eefId_) {
        switch (eefId_) {
            case static_cast<int>(EEfID::rightFoot): {
                return std::string("rf");
                break;
            }
            case static_cast<int>(EEfID::leftFoot): {
                return std::string("lf");
                break;
            }
            case static_cast<int>(EEfID::rightHand): {
                return std::string("rh");
                break;
            }
            case static_cast<int>(EEfID::leftHand): {
                return std::string("lh");
                break;
            }
            default: {
                throw std::runtime_error(
                    "End Effector ID not handled in effIdToString. \n");
            }
        }
    }
};
