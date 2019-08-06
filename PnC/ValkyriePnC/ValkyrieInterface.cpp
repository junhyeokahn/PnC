#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ValkyriePnC/TestSet/BalanceTest.hpp>
#include <PnC/ValkyriePnC/TestSet/WalkingTest.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateEstimator.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <string>

ValkyrieInterface::ValkyrieInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Valkyrie Interface");

    robot_ = new RobotSystem(
        6, THIS_COM "RobotModel/Robot/Valkyrie/ValkyrieSim_Dart.urdf");
    // robot_->printRobotInfo();
    state_estimator_ = new ValkyrieStateEstimator(robot_);
    sp_ = ValkyrieStateProvider::getStateProvider(robot_);
    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);
}

ValkyrieInterface::~ValkyrieInterface() {
    delete robot_;
    delete state_estimator_;
    delete test_;
}

void ValkyrieInterface::getCommand(void* _data, void* _command) {
    ValkyrieCommand* cmd = ((ValkyrieCommand*)_command);
    ValkyrieSensorData* data = ((ValkyrieSensorData*)_data);

    if (!Initialization_(data, cmd)) {
        state_estimator_->Update(data);
        test_->getCommand(cmd);
        CropTorque_(cmd);
    }

    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;

    ++count_;
    running_time_ = (double)(count_)*ValkyrieAux::servo_rate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = test_->getPhase();
}

void ValkyrieInterface::CropTorque_(ValkyrieCommand* cmd) {
    cmd->jtrq = myUtils::CropVector(cmd->jtrq, robot_->GetTorqueLowerLimits(),
                                    robot_->GetTorqueUpperLimits(),
                                    "clip trq in interface");
}

void ValkyrieInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/Valkyrie/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "balance_test") {
            test_ = new BalanceTest(robot_);
        } else if (test_name == "walking_test") {
            test_ = new WalkingTest(robot_);
        } else {
            printf(
                "[Valkyrie Interface] There is no test matching test with "
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

bool ValkyrieInterface::Initialization_(ValkyrieSensorData* _sensor_data,
                                        ValkyrieCommand* _command) {
    static bool test_initialized(false);
    if (!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
    }
    if (count_ < waiting_count_) {
        state_estimator_->Initialization(_sensor_data);
        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}
