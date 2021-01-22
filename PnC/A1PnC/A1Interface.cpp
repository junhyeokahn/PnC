#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
// #include <PnC/A1PnC/A1StateProvider.hpp>
// #include <PnC/A1PnC/A1StateEstimator.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
// #include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
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

    // sp_ = A1StateProvider::getStateProvider(robot_);
    // state_estimator_ = new A1StateEstimator(robot_);

    waiting_count_ = 10;

    cmd_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    cmd_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    cmd_jtrq_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    data_torque_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    stop_test_ = false;

    myUtils::color_print(myColor::BoldCyan, border);

    DataManager::GetDataManager()->RegisterData(&running_time_, DOUBLE,
                                                "running_time",1);
    DataManager::GetDataManager()->RegisterData(&cmd_jpos_, VECT,"jpos_des",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&cmd_jvel_, VECT,"jvel_des",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&cmd_jtrq__, VECT,"command",
                                                robot_->getNumActuatedDofs());
    DataManager::GetDataManager()->RegisterData(&data_torque_, VECT,"torque",
                                                robot_->getNumActuatedDofs());

    _ParameterSetting();
}

A1Interface::~A1Interface() {
    delete robot_;
    // delete test_;
}


void A1Interface::getCommand(void* _data, void* _command){
    A1Command* cmd = ((A1Command*)_command);
    A1SensorData* data = ((A1SensorData*)_data);

    if(!Initialization_(data,cmd)){
        // state_estimator_->Update(data);
        sp_->saveCurrentData();
        // data->q_act = sp_->act_q_;
        // data->qdot_act = sp_->act_qdot_;
        robot_->updateSystem(data->q, data->qdot, true);
        // test_->getCommand(cmd);
        //CropTorque_(cmd);
    }

    ++count_;
    running_time_ = (double)(count_)*A1Aux::ServoRate;
    sp_->curr_time = running_time_;
    // sp_->phase_copy = test_->getPhase();
}

void A1Interface::_ParameterSetting(){

    try{
        YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/A1/INTERFACE.yaml");

        std::string test_name = myUtils::readParameter<std::string>(cfg, "test_name");

        if(test_name == "move_test") {
            //test_ = new MoveTest(robot_);
        }
        else{
            printf(
                "[A1 Interface] There is no test matching test with " 
                "the name\n");
            exit(0);
        } 
    }
    catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
            << __FILE__ << "]" << std::endl
            << std::endl;
        exit(0);
    }
}


bool A1Interface::Initialization_(A1SensorData* _sensor_data, A1Command* _Command){
    if(!test_initialized){
        // test_->TestInitialization();
        test_initialized=true;
        DataManager::GetDataManager()->start();
    }
    return false;
}

