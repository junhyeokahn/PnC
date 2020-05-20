#pragma once

#include <PnC/ControlArchitecture.hpp>

class ValkyrieStateProvider;

// 
namespace VALKYRIE_STATES {
    constexpr int BALANCE = 0;
};  

class ValkyrieControlArchitecture : public ControlArchitecture {
   public:
    ValkyrieControlArchitecture(RobotSystem*);
    virtual ~ValkyrieControlArchitecture();
    virtual void ControlArchitectureInitialization();
    virtual void getCommand(void* _command);

   protected:
    ValkyrieStateProvider* sp_;
    void _SettingParameter();

    YAML::Node cfg_;

    // Temporary
    bool b_first_visit_;
    Controller* balance_ctrl_;

};
