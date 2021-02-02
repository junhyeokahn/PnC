#include <math.h>
#include <stdio.h>
#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoCtrlArchitecture.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoLogicInterrupt/WalkingInterruptLogic.hpp>
#include <PnC/DracoPnC/DracoStateEstimator.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
#include <PnC/Filter/Basic/filter.hpp>
#include <string>

DracoInterface::DracoInterface() : EnvInterface() {
  std::string border = "=";
  for (int i = 0; i < 79; ++i) {
    border += "=";
  }
  myUtils::color_print(myColor::BoldCyan, border);
  myUtils::pretty_constructor(0, "Draco Interface");

  robot_ =
      new RobotSystem(6, THIS_COM "RobotModel/Robot/Draco/DracoPnC_Dart.urdf");
  // robot_->printRobotInfo();
  interrupt = new InterruptLogic();

  state_estimator_ = new DracoStateEstimator(robot_);
  sp_ = DracoStateProvider::getStateProvider(robot_);
  sp_->stance_foot = DracoBodyNode::lFootCenter;

  waiting_count_ = 10;

  cmd_jtrq_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  cmd_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  cmd_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

  data_torque_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  data_temperature_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  data_motor_current_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
  rfoot_ati_ = Eigen::VectorXd::Zero(6);
  lfoot_ati_ = Eigen::VectorXd::Zero(6);

  rfoot_front_ati_ = Eigen::VectorXd::Zero(6);
  rfoot_back_ati_ =  Eigen::VectorXd::Zero(6);
  lfoot_front_ati_ =  Eigen::VectorXd::Zero(6);
  lfoot_back_ati_ =  Eigen::VectorXd::Zero(6);

  rfoot_front_est_ = Eigen::VectorXd::Zero(6);
  rfoot_back_est_ =  Eigen::VectorXd::Zero(6);
  lfoot_front_est_ =  Eigen::VectorXd::Zero(6);
  lfoot_back_est_ =  Eigen::VectorXd::Zero(6);

  Fr_ext_ = Eigen::VectorXd::Zero(6);

  x_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 2.0);
  y_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 7.5);
  z_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 250);

  x1_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 2.0);
  y1_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 7.5);
  z1_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 250);

  x2_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 2.0);
  y2_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 7.5);
  z2_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 250);

  x3_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 2.0);
  y3_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 7.5);
  z3_force_est_ = new AverageFilter(DracoAux::servo_rate, 0.030, 250);

  ((AverageFilter*)x_force_est_)->initialization(rfoot_front_ati_[3]);
  ((AverageFilter*)y_force_est_)->initialization(rfoot_front_ati_[4]);
  ((AverageFilter*)z_force_est_)->initialization(rfoot_front_ati_[5]);

  ((AverageFilter*)x1_force_est_)->initialization(rfoot_back_ati_[3]);
  ((AverageFilter*)y1_force_est_)->initialization(rfoot_back_ati_[4]);
  ((AverageFilter*)z1_force_est_)->initialization(rfoot_back_ati_[5]);

  ((AverageFilter*)x2_force_est_)->initialization(lfoot_front_ati_[3]);
  ((AverageFilter*)y2_force_est_)->initialization(lfoot_front_ati_[4]);
  ((AverageFilter*)z2_force_est_)->initialization(lfoot_front_ati_[5]);

  ((AverageFilter*)x3_force_est_)->initialization(lfoot_back_ati_[3]);
  ((AverageFilter*)y3_force_est_)->initialization(lfoot_back_ati_[4]);
  ((AverageFilter*)z3_force_est_)->initialization(lfoot_back_ati_[5]);


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
  // DataManager::GetDataManager()->RegisterData(&rfoot_ati_, VECT, "rfoot_ati",
  //                                             6);
  // DataManager::GetDataManager()->RegisterData(&lfoot_ati_, VECT, "lfoot_ati",
  //                                             6);

  DataManager::GetDataManager()->RegisterData(&rfoot_front_ati_, VECT, "rfoot_front_ati",
                                              6);
  DataManager::GetDataManager()->RegisterData(&rfoot_back_ati_, VECT, "rfoot_back_ati",
                                              6);
  DataManager::GetDataManager()->RegisterData(&lfoot_front_ati_, VECT, "lfoot_front_ati",
                                              6);
  DataManager::GetDataManager()->RegisterData(&lfoot_back_ati_, VECT, "lfoot_back_ati",
                                              6);

  DataManager::GetDataManager()->RegisterData(&rfoot_front_est_, VECT, "rfoot_front_est",
                                              6);
  DataManager::GetDataManager()->RegisterData(&rfoot_back_est_, VECT, "rfoot_back_est",
                                              6);
  DataManager::GetDataManager()->RegisterData(&lfoot_front_est_, VECT, "lfoot_front_est",
                                              6);
  DataManager::GetDataManager()->RegisterData(&lfoot_back_est_, VECT, "lfoot_back_est",
                                              6);

  DataManager::GetDataManager()->RegisterData(&Fr_ext_, VECT, "Fr_ext",
                                              6);

  // DataManager::GetDataManager()->RegisterData(&imu_acc_, VECT, "imu_acc", 3);
  // DataManager::GetDataManager()->RegisterData(&imu_angvel_, VECT,
  // "imu_angvel",
  // 3);

  _ParameterSetting();

  myUtils::color_print(myColor::BoldCyan, border);
}

DracoInterface::~DracoInterface() {
  delete state_estimator_;
  delete interrupt;
  delete control_architecture_;
  delete robot_;
  delete x_force_est_;
  delete y_force_est_;
  delete z_force_est_;
  delete x1_force_est_;
  delete y1_force_est_;
  delete z1_force_est_;
  delete x2_force_est_;
  delete y2_force_est_;
  delete z2_force_est_;
  delete x3_force_est_;
  delete y3_force_est_;
  delete z3_force_est_;

}

void DracoInterface::getCommand(void* _data, void* _command) {
  DracoCommand* cmd = ((DracoCommand*)_command);
  DracoSensorData* data = ((DracoSensorData*)_data);

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
  // rfoot_ati_ = data->rf_wrench;
  // lfoot_ati_ = data->lf_wrench;
  rfoot_front_ati_ = data->rf_front_wrench;
  x_force_est_->input(data->rf_front_wrench[3]);
  rfoot_front_ati_[3] = x_force_est_->output();
  y_force_est_->input(data->rf_front_wrench[4]);
  rfoot_front_ati_[4] = y_force_est_->output();
  z_force_est_->input(data->rf_front_wrench[5]);
  rfoot_front_ati_[5] = z_force_est_->output();  
  sp_->r_front_rf = rfoot_front_ati_;  

  rfoot_back_ati_ = data->rf_back_wrench;
  x1_force_est_->input(data->rf_back_wrench[3]);
  rfoot_back_ati_[3] = x1_force_est_->output();
  y1_force_est_->input(data->rf_back_wrench[4]);
  rfoot_back_ati_[4] = y1_force_est_->output();
  z1_force_est_->input(data->rf_back_wrench[5]);
  rfoot_back_ati_[5] = z1_force_est_->output();
  sp_->r_back_rf = rfoot_back_ati_;

  lfoot_front_ati_ = data->lf_front_wrench;
  x2_force_est_->input(data->lf_front_wrench[3]);
  lfoot_front_ati_[3] = x2_force_est_->output();
  y2_force_est_->input(data->lf_front_wrench[4]);
  lfoot_front_ati_[4] = y2_force_est_->output();
  z2_force_est_->input(data->lf_front_wrench[5]);
  lfoot_front_ati_[5] = z2_force_est_->output();  
  sp_->l_front_rf = lfoot_front_ati_;
  
  lfoot_back_ati_ = data->lf_back_wrench;
  x3_force_est_->input(data->lf_back_wrench[3]);
  lfoot_back_ati_[3] = x3_force_est_->output();
  y3_force_est_->input(data->lf_back_wrench[4]);
  lfoot_back_ati_[4] = y3_force_est_->output();
  z3_force_est_->input(data->lf_back_wrench[5]);
  lfoot_back_ati_[5] = z3_force_est_->output();
  sp_->l_back_rf = lfoot_back_ati_;

  // myUtils::pretty_print(sp_->r_front_rf, std::cout, "sp_r_front_rf");


  // imu_acc_ = data->imu_acc;
  // imu_angvel_ = data->imu_ang_vel;

  running_time_ = (double)(count_)*DracoAux::servo_rate;
  sp_->curr_time = running_time_;
  sp_->phase_copy = control_architecture_->getState();
  ++count_;
}

bool DracoInterface::_UpdateTestCommand(DracoCommand* cmd) {
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

  Eigen::VectorXd temp = cmd->Fr_estimated;
  rfoot_front_est_ = temp.segment(0,6);
  rfoot_back_est_ = temp.segment(6,6);
  lfoot_front_est_ = temp.segment(12,6);
  lfoot_back_est_ = temp.segment(18,6);

  Fr_ext_= cmd->Fr_ext;
}

bool DracoInterface::_Initialization(DracoSensorData* data, DracoCommand* cmd) {
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

void DracoInterface::_ParameterSetting() {
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Draco/INTERFACE.yaml");
    std::string test_name =
        myUtils::readParameter<std::string>(cfg, "test_name");
    if (test_name == "walking") {
      control_architecture_ = new DracoControlArchitecture(robot_);
      delete interrupt;
      interrupt = new WalkingInterruptLogic(
          static_cast<DracoControlArchitecture*>(control_architecture_));
    } else {
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
