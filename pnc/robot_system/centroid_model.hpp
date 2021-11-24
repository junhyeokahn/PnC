#pragma once

#include <stdio.h>
#include <stdexcept>
#include <string>

class CentroidModel {

    public:
        static constexpr int numEEf = 4;

        enum class EEfID {
            lfoot = 0,
            rfoot = 1,
            lhand = 2,
            rhand = 3
        };

        static bool isHand(int eEfId){
            if (eEfId == static_cast<int>(EEfID::lhand) || eEfId == static_cast<int>(EEfID::rhand)){
                return true;
                else{
                return false;}
            }
        }

        static std::string eEfIdToString(int eEfId){
            switch (eEfID){
                case static_cast<int>(EEfID::lfoot):
                    { return std::string("left foot");
                    break;
                     }
                case static_cast<int>(EEfID::rfoot):
                    { return std::string("right foot");
                    break;
                     }
                case static_cast<int>(EEfID::lhand):
                    { return std::string("left hand");
                    break;
                     }
                case static_cast<int>(EEfID::lfoot):
                    { return std::string("left foot");
                    break;
                     }
                defualt: {
                         throw std::runtime_error(
                                 "End Effector ID not handled in effIDToString. \n")
                         }
            }
        }

};
