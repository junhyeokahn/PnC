#pragma once

#include <PnC/ControlArchitecture.hpp>

class ValkyrieStateProvider;

// 
namespace VALKYRIE_STATES {
    constexpr int BALANCE = 0;
    constexpr int SWING = 1;
};  

class ValkyrieControlArchitecture : public ControlArchitecture {
   public:
    ValkyrieControlArchitecture(RobotSystem*);
    virtual ~ValkyrieControlArchitecture();
    virtual void ControlArchitectureInitialization();
    virtual void getCommand(void* _command);

   protected:
    ValkyrieStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    void _SettingParameter();

    YAML::Node cfg_;

    // Temporary
    bool b_first_visit_;
    std::vector<Controller*> state_list_;

    Controller* balance_ctrl_;
    Controller* swing_ctrl_;

};
