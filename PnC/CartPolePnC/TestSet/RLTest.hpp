#pragma once

#include <PnC/Test.hpp>

class RobotSystem;

enum RL_TEST_PHASE { NN = 0, NUM_RL_TEST = 1 };

class RLTest : public Test {
   public:
    RLTest(RobotSystem* _robot);
    RLTest(RobotSystem* _robot, int mpi_idx, int env_idx);
    virtual ~RLTest();

    virtual void TestInitialization();

   protected:
    void _ParameterSetting();
    virtual int _NextPhase(const int& phase);

    Controller* learning_ctrl_;
    Controller* policy_ctrl_;

    YAML::Node cfg_;
    bool b_learning_;

    int mpi_idx_;
    int env_idx_;
};
