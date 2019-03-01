#include <math.h>
#include <stdio.h>
#include <PnC/FixedDracoPnC/FixedDracoInterface.hpp>
#include <PnC/FixedDracoPnC/TestSet/JointTest.hpp>
//#include <PnC/FixedDracoPnC/FixedDracoDefinition.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <string>

FixedDracoInterface::FixedDracoInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Fixed Draco Interface");

    robot_ = new RobotSystem(
        0, THIS_COM "RobotModel/Robot/Draco/FixedDracoPnC_Dart.urdf");
    // robot_->printRobotInfo();

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);
}

FixedDracoInterface::~FixedDracoInterface() { delete robot_; }

void FixedDracoInterface::getCommand(void* _data, void* _command) {
    FixedDracoCommand* cmd = ((FixedDracoCommand*)_command);
    FixedDracoSensorData* data = ((FixedDracoSensorData*)_data);

    static bool b_initialized(false);
    if (!b_initialized) {
        test_->TestInitialization();
        b_initialized = true;
    }

    Eigen::VectorXd q = data->q;
    Eigen::VectorXd qdot = data->qdot;
    robot_->updateSystem(q, qdot, false);
    test_->getCommand(cmd);

    ++count_;
}

void FixedDracoInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/FixedDraco/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "joint_test") {
            test_ = new JointTest(robot_);
        } else {
            printf(
                "[Fixed Draco Interface] There is no test matching test with "
                "the name\n");
            exit(0);
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
