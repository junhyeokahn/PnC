#include <Configuration.h>
#include <stdio.h>
#include <PnC/CartPolePnC/CartPoleDefinition.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <PnC/CartPolePnC/TestSet/BasicTest.hpp>
#include <PnC/CartPolePnC/TestSet/PolicyTest.hpp>
#include <PnC/CartPolePnC/TestSet/RLTest.hpp>
#include <RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <chrono>
#include <thread>

CartPoleInterface::CartPoleInterface() : Interface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    mpi_idx_ = 0;
    env_idx_ = 0;
    b_learning = false;
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "CartPole Interface");

    robot_ = new RobotSystem(
        1, THIS_COM "RobotSystem/RobotModel/Robot/CartPole/CartPole.urdf");
    // robot_->printRobotInfo();

    ParameterSetting_();

    myUtils::color_print(myColor::BoldCyan, border);
}

CartPoleInterface::CartPoleInterface(int mpi_idx, int env_idx) : Interface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(
        0, "CartPole Interface ( MPI : " + std::to_string(mpi_idx) +
               ", ENV : " + std::to_string(env_idx) + " )");

    mpi_idx_ = mpi_idx;
    env_idx_ = env_idx;
    b_learning = true;

    robot_ = new RobotSystem(
        1, THIS_COM "RobotSystem/RobotModel/Robot/CartPole/CartPole.urdf");
    // robot_->printRobotInfo();

    ParameterSetting_();

    myUtils::color_print(myColor::BoldCyan, border);
}

CartPoleInterface::~CartPoleInterface() {
    delete test_;
    delete robot_;
}

void CartPoleInterface::getCommand(void* _data, void* _cmd) {
    static bool test_initialized(false);
    if (!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
    }

    CartPoleCommand* cmd = ((CartPoleCommand*)_cmd);
    CartPoleSensorData* data = ((CartPoleSensorData*)_data);

    robot_->updateSystem(data->q, data->qdot, false);
    test_->getCommand(cmd);

    running_time_ = (double)(count_)*CartPoleAux::ServoRate;
    ++count_;
}

void CartPoleInterface::ParameterSetting_() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/CartPole/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "rl_test") {
#if HAS_RL_DEP
            if (b_learning) {
                test_ = new RLTest(robot_, mpi_idx_, env_idx_);
            } else {
                test_ = new RLTest(robot_);
            }
#else
            std::cout << "[Error] Dependancies for Reinforcement Learning is "
                         "not found"
                      << std::endl;
#endif
        } else if (test_name == "policy_test") {
            test_ = new PolicyTest(robot_);
        } else if (test_name == "basic_test") {
            test_ = new BasicTest(robot_);
        } else {
            printf(
                "[Cartpole Interface] There is no test matching test with the "
                "name\n");
            exit(0);
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
