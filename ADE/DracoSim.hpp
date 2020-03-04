#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <Configuration.h>
#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <thread>

class DracoSim {
private:
    dart::gui::osg::Viewer viewer;
    //Threading
    std::thread thread_;
    std::atomic<bool> exit;
    void StartThread();
public:
    //Initialize the sim with a new PnC
    DracoSim();
    //Start the simulator and visuals in a new thread
    void StartSim(EnvInterface* interface, EnvInterface* arm_interface);
    //Stop the simulator and join the thread it was running in
    void StopSim();
};


