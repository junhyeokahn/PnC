#include <math.h>
#include <stdio.h>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateEstimator.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/TestSet/RLWalkingTest.hpp>
#include <PnC/AtlasPnC/TestSet/WalkingTest.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <string>

AtlasInterface::AtlasInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Atlas Interface");

    mpi_idx_ = 0;
    env_idx_ = 0;
    b_learning_ = false;

    robot_ = new RobotSystem(
        6, THIS_COM "RobotModel/Robot/Atlas/AtlasSim_Dart.urdf");
    // robot_->printRobotInfo();
    state_estimator_ = new AtlasStateEstimator(robot_);
    sp_ = AtlasStateProvider::getStateProvider(robot_);
    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Atlas::n_adof);

    _ParameterSetting();

    DataManager* dm = DataManager::GetDataManager();
    dm->RegisterData(&cmd_jpos_, VECT, "jpos_des", Atlas::n_adof);
    dm->RegisterData(&cmd_jvel_, VECT, "jvel_des", Atlas::n_adof);
    dm->RegisterData(&cmd_jtrq_, VECT, "command", Atlas::n_adof);

    myUtils::color_print(myColor::BoldCyan, border);
}

AtlasInterface::AtlasInterface(int mpi_idx, int env_idx) : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(
        0, "Atlas Interface ( MPI : " + std::to_string(mpi_idx) +
               ", ENV : " + std::to_string(env_idx) + " )");

    mpi_idx_ = mpi_idx;
    env_idx_ = env_idx;
    b_learning_ = true;

    robot_ = new RobotSystem(
        6, THIS_COM "RobotModel/Robot/Atlas/AtlasSim_Dart.urdf");
    // robot_->printRobotInfo();
    state_estimator_ = new AtlasStateEstimator(robot_);
    sp_ = AtlasStateProvider::getStateProvider(robot_);
    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Atlas::n_adof);

    _ParameterSetting();

    DataManager* dm = DataManager::GetDataManager();
    dm->RegisterData(&cmd_jpos_, VECT, "jpos_des", Atlas::n_adof);
    dm->RegisterData(&cmd_jvel_, VECT, "jvel_des", Atlas::n_adof);
    dm->RegisterData(&cmd_jtrq_, VECT, "command", Atlas::n_adof);

    myUtils::color_print(myColor::BoldCyan, border);
}

AtlasInterface::~AtlasInterface() {
    delete robot_;
    delete state_estimator_;
    delete test_;
}

void AtlasInterface::getCommand(void* _data, void* _command) {
    AtlasCommand* cmd = ((AtlasCommand*)_command);
    AtlasSensorData* data = ((AtlasSensorData*)_data);

    if (!Initialization_(data, cmd)) {
        state_estimator_->Update(data);
        test_->getCommand(cmd);
        CropTorque_(cmd);
    }

    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;

    ++count_;
    running_time_ = (double)(count_)*AtlasAux::servo_rate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = test_->getPhase();
}

void AtlasInterface::CropTorque_(AtlasCommand* cmd) {
    cmd->jtrq = myUtils::CropVector(cmd->jtrq, robot_->GetTorqueLowerLimits(),
                                    robot_->GetTorqueUpperLimits(),
                                    "clip trq in interface");
}

void AtlasInterface::_ParameterSetting() {
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Atlas/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "walking_test") {
            test_ = new WalkingTest(robot_);
        } else if (test_name == "rl_walking_test") {
#if HAS_RL_DEP
            if (!b_learning_) {
                test_ = new RLWalkingTest(robot_);
            } else {
                test_ = new RLWalkingTest(robot_, mpi_idx_, env_idx_);
            }
#else
            std::cout << "[Error] Dependancies for Reinforcement Learning in "
                         "not found"
                      << std::endl;
            exit(0);
#endif
        } else {
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

bool AtlasInterface::Initialization_(AtlasSensorData* _sensor_data,
                                     AtlasCommand* _command) {
    static bool test_initialized(false);
    if (!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
    }
    if (count_ < waiting_count_) {
        SetStopCommand_(_sensor_data, _command);
        state_estimator_->Initialization(_sensor_data);
        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

void AtlasInterface::SetStopCommand_(AtlasSensorData* _sensor_data,
                                     AtlasCommand* _command) {
    for (int i(0); i < Atlas::n_adof; ++i) {
        _command->jtrq[i] = 0.;
        _command->q[i] = _sensor_data->q[i];
        _command->qdot[i] = 0.;
    }
}

Eigen::Isometry3d AtlasInterface::GetTargetIso() {
    Eigen::Isometry3d target_iso = Eigen::Isometry3d::Identity();
    Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 2; ++i) {
        target_pos[i] = sp_->des_location[i];
    }
    target_iso.translation() = target_pos;
    target_iso.linear() = sp_->des_quat.toRotationMatrix();
    return target_iso;
}
