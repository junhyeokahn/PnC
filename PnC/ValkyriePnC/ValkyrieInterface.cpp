#include <math.h>
#include <stdio.h>
#include <string>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ValkyriePnC/ValkyrieCtrlArchitecture/ValkyrieControlArchitecture.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieLogicInterrupt/WalkingInterruptLogic.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateEstimator.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

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

  // Initialize empty interrupt class
  interrupt_ = new InterruptLogic();

  sp_->stance_foot = ValkyrieBodyNode::leftCOP_Frame;

  count_ = 0;
  waiting_count_ = 2;
  cmd_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
  cmd_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
  cmd_jtrq_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);

  _ParameterSetting();

  myUtils::color_print(myColor::BoldCyan, border);

  DataManager* data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&cmd_jpos_, VECT, "jpos_des", Valkyrie::n_adof);
  data_manager->RegisterData(&cmd_jvel_, VECT, "jvel_des", Valkyrie::n_adof);
  data_manager->RegisterData(&cmd_jtrq_, VECT, "command", Valkyrie::n_adof);
}

ValkyrieInterface::~ValkyrieInterface() {
  delete robot_;
  delete state_estimator_;
  delete interrupt_;
  delete control_architecture_;
}

void ValkyrieInterface::getCommand(void* _data, void* _command) {
  ValkyrieCommand* cmd = ((ValkyrieCommand*)_command);
  ValkyrieSensorData* data = ((ValkyrieSensorData*)_data);

  if (!Initialization_(data, cmd)) {
    state_estimator_->Update(data);
    interrupt_->processInterrupts();
    control_architecture_->getCommand(cmd);
    CropTorque_(cmd);
  }

  cmd_jtrq_ = cmd->jtrq;
  cmd_jvel_ = cmd->qdot;
  cmd_jpos_ = cmd->q;

  ++count_;
  running_time_ = (double)(count_)*ValkyrieAux::servo_rate;
  sp_->curr_time = running_time_;

  sp_->phase_copy = control_architecture_->getState();
}

void ValkyrieInterface::CropTorque_(ValkyrieCommand* cmd) {
  cmd->jtrq =
      myUtils::CropVector(cmd->jtrq, robot_->GetTorqueLowerLimits().segment(
                                         Valkyrie::n_vdof, Valkyrie::n_adof),
                          robot_->GetTorqueUpperLimits().segment(
                              Valkyrie::n_vdof, Valkyrie::n_adof),
                          "clip trq in interface");
}

void ValkyrieInterface::_ParameterSetting() {
  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Valkyrie/INTERFACE.yaml");
    std::string test_name =
        myUtils::readParameter<std::string>(cfg, "test_name");
    if (test_name == "walking") {
      control_architecture_ = new ValkyrieControlArchitecture(robot_);
      delete interrupt_;
      interrupt_ = new WalkingInterruptLogic(
          static_cast<ValkyrieControlArchitecture*>(control_architecture_));
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
    control_architecture_->ControlArchitectureInitialization();
    test_initialized = true;
  }
  if (count_ < waiting_count_) {
    state_estimator_->Initialization(_sensor_data);
    DataManager::GetDataManager()->start();
    return true;
  }
  return false;
}
