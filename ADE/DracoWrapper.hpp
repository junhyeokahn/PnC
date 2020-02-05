#include <PnC/DracoPnC/DracoInterface.hpp>
#include <ADE/DracoSim.hpp>
#include <Configuration.h>
#include <Simulator/Dart/Draco/DracoWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <thread>

class DracoWrapper {
private:
    //PnC, sim, and robot members
    DracoInterface* interface_;
    DracoSim* simulator_;

    //Logical Safety
    bool running_;

public:
    DracoWrapper();

    //Set up the DracoWrapper to create the PnC instance and pass it down into the sim and ros nodelet to centralize commands
    void Initialize();
    //Accept a command from ADE to forward to the PnC as long as Initialize has been called
    void SetWalkCommand(double ft_length, double r_ft_width, double l_ft_width,
                        double ori_inc, int num_step);
    //Shutdown the sim and ros nodelet and destruct the PnC instance
    void Shutdown();
};


