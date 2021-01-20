#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
// #include <PnC/A1PnC/A1StateProvider.hpp>
#include <string>

/*
A1Interface::A1Interface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "A1 Interface");

    robot_ = new RobotSystem(
        4, THIS_COM "RobotModel/Robot/A1/a1_sim.urdf");
     //robot_->printRobotInfo();
    // sp_ = A1StateProvider::getStateProvider(robot_);

    count_ = 0;
    //waiting_count_ = 2;
    test_initialized = false;
    cmd_jpos_ = Eigen::VectorXd::Zero(A1::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(A1::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(A1::n_adof);

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&running_time_,DOUBLE,"time",1);
    data_manager->RegisterData(&cmd_jpos_, VECT, "jpos_des", A1::n_adof);
    data_manager->RegisterData(&cmd_jvel_, VECT, "jvel_des", A1::n_adof);
    data_manager->RegisterData(&cmd_jtrq_, VECT, "jtrq_des", A1::n_adof);
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
*/
