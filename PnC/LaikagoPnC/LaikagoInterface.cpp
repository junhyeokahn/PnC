#include <math.h>
#include <stdio.h>
#include <PnC/LaikagoPnC/LaikagoCtrlArchitecture/LaikagoCtrlArchitecture.hpp>
#include <PnC/LaikagoPnC/LaikagoDefinition.hpp>
#include <PnC/LaikagoPnC/LaikagoInterface.hpp>
#include <PnC/LaikagoPnC/LaikagoLogicInterrupt/WalkingInterruptLogic.hpp>
#include <PnC/LaikagoPnC/LaikagoStateEstimator.hpp>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
#include <string>

LaikagoInterface::LaikagoInterface() : EnvInterface() {
  std::string border = "=";
  for (int i = 0; i < 79; ++i) {
    border += "=";
  }
  myUtils::color_print(myColor::BoldCyan, border);
  myUtils::pretty_constructor(0, "Laikago Interface");

  robot_ =
      new RobotSystem(6, THIS_COM "RobotModel/Robot/Laikago/Laikago.urdf");
  // robot_->printRobotInfo();
  interrupt = new InterruptLogic();

  state_estimator_ = new LaikagoStateEstimator(robot_);
  sp_ = LaikagoStateProvider::getStateProvider(robot_);
  sp_->stance_foot = LaikagoBodyNode::FL_foot;

  waiting_count_ = 10;

  cmd_jtrq_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  cmd_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  cmd_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

  data_torque_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  data_temperature_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  data_motor_current_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  rfoot_ati_ = Eigen::VectorXd::Zero(6);
  lfoot_ati_ = Eigen::VectorXd::Zero(6);
  // imu_acc_ = Eigen::VectorXd::Zero(3);
  // imu_angvel_ = Eigen::VectorXd::Zero(3);

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
  // DataManager::GetDataManager()->RegisterData(&imu_acc_, VECT, "imu_acc", 3);
  // DataManager::GetDataManager()->RegisterData(&imu_angvel_, VECT,
  // "imu_angvel",
  // 3);

  _ParameterSetting();

  myUtils::color_print(myColor::BoldCyan, border);
}

LaikagoInterface::~LaikagoInterface() {
  delete state_estimator_;
  delete interrupt;
  delete control_architecture_;
  delete robot_;
}

void LaikagoInterface::getCommand(void* _data, void* _command) {
  LaikagoCommand* cmd = ((LaikagoCommand*)_command);
  LaikagoSensorData* data = ((LaikagoSensorData*)_data);

  if (!_Initialization(data, cmd)) {
    state_estimator_->update(data);
    interrupt->processInterrupts();
    control_architecture_->getCommand(cmd);

    stop_test_ = _UpdateTestCommand(cmd);
    if (stop_test_) {
      if (count_ % 100 == 0) {
        myUtils::color_print(myColor::Yellow, "[Setting Stop Command]");
      }
      _SetStopCommand(data, cmd);
    } else {
      _CopyCommand(cmd);
    }
  }

  // Save Data
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    data_torque_[i] = data->jtrq[i];
    data_temperature_[i] = data->temperature[i];
    data_motor_current_[i] = data->motor_current[i];
  }
  rfoot_ati_ = data->rf_wrench;
  lfoot_ati_ = data->lf_wrench;
  // imu_acc_ = data->imu_acc;
  // imu_angvel_ = data->imu_ang_vel;

  running_time_ = (double)(count_)*LaikagoAux::servo_rate;
  sp_->curr_time = running_time_;
  sp_->phase_copy = control_architecture_->getState();
  ++count_;
}

bool LaikagoInterface::_UpdateTestCommand(LaikagoCommand* cmd) {
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

void LaikagoInterface::_SetStopCommand(LaikagoSensorData* data, LaikagoCommand* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = 0.;
    cmd->q[i] = data->q[i];
    cmd->qdot[i] = 0.;
  }
}

void LaikagoInterface::_CopyCommand(LaikagoCommand* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = cmd_jtrq_[i];
    cmd->q[i] = cmd_jpos_[i];
    cmd->qdot[i] = cmd_jvel_[i];
  }
}

bool LaikagoInterface::_Initialization(LaikagoSensorData* data, LaikagoCommand* cmd) {
  static bool test_initialized(false);
  if (!test_initialized) {
    control_architecture_->ControlArchitectureInitialization();
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

void LaikagoInterface::_ParameterSetting() {
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Laikago/INTERFACE.yaml");
    std::string test_name =
        myUtils::readParameter<std::string>(cfg, "test_name");
    if (test_name == "walking") {
      control_architecture_ = new LaikagoControlArchitecture(robot_);
      delete interrupt;
      interrupt = new WalkingInterruptLogic(
          static_cast<LaikagoControlArchitecture*>(control_architecture_));
    } else {
      printf(
          "[Laikago Interface] There is no matching test with the "
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
