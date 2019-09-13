#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
//#include <PnC/DracoPnC/TestSet/BalanceTest.hpp>
#include <PnC/DracoPnC/TestSet/WalkingTest.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateEstimator.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <string>

DracoInterface::DracoInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Draco Interface");

    robot_ = new RobotSystem(
        6, THIS_COM "RobotModel/Robot/Draco/DracoSim_Dart.urdf");
    // robot_->printRobotInfo();
    state_estimator_ = new DracoStateEstimator(robot_);
    sp_ = DracoStateProvider::getStateProvider(robot_);

    sp_->stance_foot = DracoBodyNode::lFootCenter;

    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(Draco::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Draco::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Draco::n_adof);

    prev_planning_moment_ = 0.;

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&cmd_jpos_, VECT, "jpos_des", Draco::n_adof);
    data_manager->RegisterData(&cmd_jvel_, VECT, "jvel_des", Draco::n_adof);
    data_manager->RegisterData(&cmd_jtrq_, VECT, "command", Draco::n_adof);
}

DracoInterface::~DracoInterface() {
    delete robot_;
    delete state_estimator_;
    delete test_;
}

void DracoInterface::getCommand(void* _data, void* _command) {

//TEST
    std::cout << "rfoot center" << std::endl;
    std::cout << robot_->getBodyNodeIsometry("rFootCenter").translation() << std::endl;
    std::cout << "lfoot center" << std::endl;
    std::cout << robot_->getBodyNodeIsometry("lFootCenter").translation() << std::endl;
    std::cout << "com pos" << std::endl;
    std::cout << robot_->getCoMPosition() << std::endl;
//TEST

    DracoCommand* cmd = ((DracoCommand*)_command);
    DracoSensorData* data = ((DracoSensorData*)_data);

    if (!Initialization_(data, cmd)) {
        state_estimator_->Update(data);
        test_->getCommand(cmd);
        CropTorque_(cmd);
    }

    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;

    ++count_;
    running_time_ = (double)(count_)*DracoAux::ServoRate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = test_->getPhase();
}

void DracoInterface::CropTorque_(DracoCommand* cmd) {
    // cmd->jtrq = myUtils::CropVector(cmd->jtrq,
    // robot_->GetTorqueLowerLimits(), robot_->GetTorqueUpperLimits(), "clip trq
    // in interface");
}

void DracoInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/Draco/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        //if (test_name == "balance_test") {
            //test_ = new BalanceTest(robot_);
        //} 
        if (test_name == "walking_test") {
            test_ = new WalkingTest(robot_);
        } else {
            printf(
                "[Draco Interface] There is no test matching test with "
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

bool DracoInterface::Initialization_(DracoSensorData* _sensor_data,
                                        DracoCommand* _command) {
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

bool DracoInterface::IsTrajectoryUpdated() {
    if (prev_planning_moment_ == sp_->planning_moment) {
        prev_planning_moment_ = sp_->planning_moment;
        return false;
    } else {
        prev_planning_moment_ = sp_->planning_moment;
        return true;
    }
}

void DracoInterface::GetCoMTrajectory(
    std::vector<Eigen::VectorXd>& com_des_list) {
    com_des_list = sp_->com_des_list;
}
void DracoInterface::GetContactSequence(
    std::vector<Eigen::Isometry3d>& foot_target_list) {
    foot_target_list = sp_->foot_target_list;
}

void DracoInterface::Walk(double ft_length, double r_ft_width,
                             double l_ft_width, double ori_inc, int num_step) {
    if (sp_->b_walking) {
        std::cout
            << "Still Walking... Please Wait Until Ongoing Walking is Done"
            << std::endl;
    } else {
        sp_->b_walking = true;
        sp_->ft_length = ft_length;
        sp_->r_ft_width = r_ft_width;
        sp_->l_ft_width = l_ft_width;
        sp_->ft_ori_inc = ori_inc;
        sp_->num_total_step = num_step;
        sp_->num_residual_step = num_step;

        ((WalkingTest*)test_)->ResetWalkingParameters();
        ((WalkingTest*)test_)->InitiateWalkingPhase();
    }
}
