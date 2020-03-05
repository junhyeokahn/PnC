#include <math.h>
#include <stdio.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateEstimator.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
#include <string>

DracoInterface::DracoInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Draco Interface");

    robot_ =
        // new RobotSystem(6, THIS_COM
        // "RobotModel/Robot/Draco/DracoPnCOld.urdf");
        new RobotSystem(6,
                        THIS_COM "RobotModel/Robot/Draco/DracoPnC_Dart.urdf");
    // robot_->printRobotInfo();

    test_cmd_ = new DracoCommand();
    test_cmd_->turn_off = false;
    test_cmd_->q = Eigen::VectorXd::Zero(10);
    test_cmd_->qdot = Eigen::VectorXd::Zero(10);
    test_cmd_->jtrq = Eigen::VectorXd::Zero(10);

    state_estimator_ = new DracoStateEstimator(robot_);
    sp_ = DracoStateProvider::getStateProvider(robot_);

    waiting_count_ = 10;

    cmd_jtrq_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    cmd_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    cmd_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    data_torque_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    data_temperature_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    data_motor_current_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    rfoot_ati_ = Eigen::VectorXd::Zero(6);
    lfoot_ati_ = Eigen::VectorXd::Zero(6);

    stop_test_ = false;

    DataManager::GetDataManager()->RegisterData(&running_time_, DOUBLE,
                                                "running_time");
    DataManager::GetDataManager()->RegisterData(&cmd_jpos_, VECT, "jpos_des",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&cmd_jvel_, VECT, "jvel_des",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&cmd_jtrq_, VECT, "command",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&data_torque_, VECT, "torque",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(
        &data_temperature_, VECT, "temperature", robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&data_motor_current_, VECT,
                                                "motor_current",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&rfoot_ati_, VECT, "rfoot_ati",
                                                6);
    DataManager::GetDataManager()->RegisterData(&lfoot_ati_, VECT, "lfoot_ati",
                                                6);

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);
}

DracoInterface::~DracoInterface() {
    delete test_;
    delete test_cmd_;
    delete state_estimator_;
    delete robot_;
}

void DracoInterface::getCommand(void* _data, void* _command) {
    DracoCommand* cmd = ((DracoCommand*)_command);
    DracoSensorData* data = ((DracoSensorData*)_data);

    if (!_Initialization(data, cmd)) {
        state_estimator_->update(data);
        test_->getCommand(test_cmd_);

        stop_test_ = _UpdateTestCommand(test_cmd_);
        if (stop_test_) {
            _SetStopCommand(data, cmd);
        } else {
            _CopyCommand(cmd);
        }
        // _CopyCommand(cmd);
    }

    // Save Data
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        data_torque_[i] = data->jtrq[i];
        data_temperature_[i] = data->temperature[i];
        data_motor_current_[i] = data->motor_current[i];
    }
    rfoot_ati_ = data->rfoot_ati;
    lfoot_ati_ = data->lfoot_ati;

    running_time_ = (double)(count_)*DracoAux::ServoRate;
    ++count_;
    sp_->curr_time = running_time_;
    sp_->phase_copy = test_->getPhase();
}

bool DracoInterface::_UpdateTestCommand(DracoCommand* test_cmd) {
    bool over_limit(false);
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        // JPos limit check
        if (test_cmd->q[i] > jpos_max_[i])
            cmd_jpos_[i] = jpos_max_[i];
        else if (test_cmd->q[i] < jpos_min_[i])
            cmd_jpos_[i] = jpos_min_[i];
        else
            cmd_jpos_[i] = test_cmd->q[i];

        // Velocity limit
        if (test_cmd->qdot[i] > jvel_max_[i])
            over_limit = true;
        else if (test_cmd->qdot[i] < jvel_min_[i])
            over_limit = true;
        else
            cmd_jvel_[i] = test_cmd->qdot[i];

        // Torque limit
        if (test_cmd->jtrq[i] > jtrq_max_[i]) {
            // over_limit = true;
            test_cmd->jtrq[i] = jtrq_max_[i];
            cmd_jtrq_[i] = test_cmd->jtrq[i];
        } else if (test_cmd->jtrq[i] < jtrq_min_[i]) {
            // over_limit = true;
            test_cmd->jtrq[i] = jtrq_min_[i];
            cmd_jtrq_[i] = test_cmd->jtrq[i];
        } else {
            cmd_jtrq_[i] = test_cmd->jtrq[i];
        }
    }
    return over_limit;
}

void DracoInterface::_SetStopCommand(DracoSensorData* data, DracoCommand* cmd) {
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        cmd->jtrq[i] = 0.;
        cmd->q[i] = data->q[i];
        cmd->qdot[i] = 0.;
    }
}

void DracoInterface::_CopyCommand(DracoCommand* cmd) {
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        cmd->jtrq[i] = cmd_jtrq_[i];
        cmd->q[i] = cmd_jpos_[i];
        cmd->qdot[i] = cmd_jvel_[i];
    }
}

bool DracoInterface::_Initialization(DracoSensorData* data, DracoCommand* cmd) {
    static bool test_initialized(false);
    if (!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
    }
    if (count_ < waiting_count_) {
        _SetStopCommand(data, cmd);
        state_estimator_->initialization(data);
        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

void DracoInterface::_ParameterSetting() {
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Draco/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "joint_test") {
            test_ = new JointTest(robot_);
        } else if (test_name == "body_test") {
            test_ = new BodyTest(robot_);
        } else if (test_name == "walking_test") {
            test_ = new WalkingTest(robot_);
        } else if (test_name == "balancing_test") {
            test_ = new BalancingTest(robot_);
        } else if (test_name == "mpc_test") {
            test_ = new MPCTest(robot_);
        } else if (test_name == "mpc_stand_test") {
            test_ = new MPCStandTest(robot_);
        } else if (test_name == "mpc_walk_test") {
            test_ = new MPCWalkTest(robot_);
        } else if (test_name == "dcm_walk_test") {
            test_ = new DCMWalkTest(robot_);
        }        
        else {
            printf(
                "[Draco Interface] There is no matching test with the "
                "name\n");
            exit(0);
        }
        myUtils::readParameter(cfg, "jpos_max", jpos_max_);
        myUtils::readParameter(cfg, "jpos_min", jpos_min_);
        myUtils::readParameter(cfg, "jvel_max", jvel_max_);
        myUtils::readParameter(cfg, "jvel_min", jvel_min_);
        myUtils::readParameter(cfg, "jtrq_max", jtrq_max_);
        myUtils::readParameter(cfg, "jtrq_min", jtrq_min_);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
