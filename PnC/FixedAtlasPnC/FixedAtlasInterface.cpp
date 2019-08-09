#include <math.h>
#include <stdio.h>
#include <PnC/FixedAtlasPnC/FixedAtlasInterface.hpp>
#include <PnC/FixedAtlasPnC/TestSet/JointTest.hpp>
//#include <PnC/FixedAtlasPnC/TestSet/OSCTest.hpp>
#include <PnC/FixedAtlasPnC/FixedAtlasDefinition.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <string>

FixedAtlasInterface::FixedAtlasInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Fixed Atlas Interface");

    robot_ = new RobotSystem(
        0, THIS_COM "RobotModel/Robot/Atlas/FixedAtlasSim_Dart.urdf");
        //robot_->printRobotInfo();

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);
}

FixedAtlasInterface::~FixedAtlasInterface() { 
    delete robot_;
    delete test_;
}

void FixedAtlasInterface::getCommand(void* _data, void* _command) {
    FixedAtlasCommand* cmd = ((FixedAtlasCommand*)_command);
    FixedAtlasSensorData* data = ((FixedAtlasSensorData*)_data);

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

void FixedAtlasInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/FixedAtlas/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "joint_test") {
            test_ = new JointTest(robot_);
        } 
        //if (test_name == "OSC_test") {
            //test_ = new OSCTest(robot_);
        //} 
        else {
            printf(
                "[Atlas Interface] There is no test matching test with "
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
