#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/Scorpio2PnC/ScorpioInterface.hpp>
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
    //Simulation Helpers
    dart::gui::osg::Viewer viewer;

    //Threading
    std::thread thread_;
    std::atomic<bool> exit;
    void StartThread();
public:
    osg::ref_ptr<ScorpioWorldNode> node;
    //Initialize the sim with a new PnC
    DracoSim();
    //Start the simulator and visuals in a new thread
    void StartSim(DracoInterface* draco, ScorpioInterface* arm1, Scorpio2Interface* arm2);
    //Stop the simulator and join the thread it was running in
    void StopSim();
};


