#include <Eigen/Dense>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>


class GaitScheduler{

    GaitScheduler(RobotSystem* robot);

    ~GaitScheduler(){};

    void step();

   protected:
    _ParameterSetting();


    RobotSystem* robot_;
    A1StateProvider* sp_;



}
