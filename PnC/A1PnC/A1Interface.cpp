#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/A1PnC/A1StateEstimator.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1LogicInterrupt/WalkingInterruptLogic.hpp>
#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <string>


A1Interface::A1Interface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "A1 Interface");

    robot_ = new RobotSystem(6, THIS_COM "RobotModel/Robot/A1/a1_sim.urdf");
    //robot_->printRobotInfo();
    interrupt = new InterruptLogic();

    sp_ = A1StateProvider::getStateProvider(robot_);
    state_estimator_ = new A1StateEstimator(robot_);

    waiting_count_ = 10;

    cmd_jpos_ = Eigen::VectorXd::Zero(12);
    cmd_jvel_ = Eigen::VectorXd::Zero(12);
    cmd_jtrq_ = Eigen::VectorXd::Zero(12);

    data_torque_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    stop_test_ = false;

    myUtils::color_print(myColor::BoldCyan, border);

    DataManager::GetDataManager()->RegisterData(&running_time_, DOUBLE,
                                                "running_time",1);
    DataManager::GetDataManager()->RegisterData(&cmd_jpos_, VECT,"jpos_des",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&cmd_jvel_, VECT,"jvel_des",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&cmd_jtrq_, VECT,"command torque",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&data_torque_, VECT,"actual torque",
                                                robot_->getNumActuatedDofs());

    _ParameterSetting();
}

A1Interface::~A1Interface() {
    delete robot_;
    delete interrupt;
    // delete test_;
}


void A1Interface::getCommand(void* _data, void* _command){
    A1Command* cmd = ((A1Command*)_command);
    A1SensorData* data = ((A1SensorData*)_data);

    if(!(_Initialization(data,cmd))){
        state_estimator_->Update(data);
        interrupt->processInterrupts();
        control_architecture_->getCommand(cmd);
    }
    // Save Data
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
      data_torque_[i] = data->jtrq[i];
    }

    running_time_ = (double)(count_)*A1Aux::servo_rate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = control_architecture_->getState();
    ++count_;
}

bool A1Interface::_UpdateTestCommand(A1Command* cmd) {
  bool over_limit(false);

  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    // JPos limit check
    if (cmd->q[i] > jpos_max_[i])
      cmd_jpos_[i] = jpos_max_[i];
    else if (cmd->q[i] < jpos_min_[i])
      cmd_jpos_[i] = jpos_min_[i];
    else
      cmd_jpos_[i] = cmd->q[i];

    // Velocity limit
    if (cmd->qdot[i] > jvel_max_[i]) {
      // over_limit = true;
    } else if (cmd->qdot[i] < jvel_min_[i]) {
      // over_limit = true;
    } else
      cmd_jvel_[i] = cmd->qdot[i];

    // Torque limit
    if (cmd->jtrq[i] > jtrq_max_[i]) {
      // over_limit = true;
      cmd->jtrq[i] = jtrq_max_[i];
      cmd_jtrq_[i] = cmd->jtrq[i];
    } else if (cmd->jtrq[i] < jtrq_min_[i]) {
      // over_limit = true;
      cmd->jtrq[i] = jtrq_min_[i];
      cmd_jtrq_[i] = cmd->jtrq[i];
    } else {
      cmd_jtrq_[i] = cmd->jtrq[i];
    }
  }
  return over_limit;
}

void A1Interface::_SetStopCommand(A1SensorData* data, A1Command* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = 0.;
    cmd->q[i] = data->q[i];
    cmd->qdot[i] = 0.;
  }
}

void A1Interface::_CopyCommand(A1Command* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = cmd_jtrq_[i];
    cmd->q[i] = cmd_jpos_[i];
    cmd->qdot[i] = cmd_jvel_[i];
  }
}

bool A1Interface::_Initialization(A1SensorData* data, A1Command* cmd) {
  static bool test_initialized(false);
  if (!test_initialized) {
    control_architecture_->ControlArchitectureInitialization();
    test_initialized = true;
  }
  if (count_ < waiting_count_) {
    _SetStopCommand(data, cmd);
    state_estimator_->Initialization(data);
    DataManager::GetDataManager()->start();
    return true;
  }
  return false;
}

void A1Interface::_ParameterSetting() {
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/A1/INTERFACE.yaml");
    std::string test_name =
        myUtils::readParameter<std::string>(cfg, "test_name");
    if (test_name == "balancing") {
      control_architecture_ = new A1ControlArchitecture(robot_);
      delete interrupt;
      interrupt = new WalkingInterruptLogic(
                static_cast<A1ControlArchitecture*>(control_architecture_));
    } else {
      printf(
          "[A1 Interface] There is no matching test with the "
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
