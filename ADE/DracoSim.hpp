#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <Configuration.h>
#include <ADE/ScorpioWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <thread>

class DracoSim {
private:
    //Simulation Helpers
    osg::ref_ptr<ScorpioWorldNode> world_;
    dart::gui::osg::Viewer viewer;

    //Threading
    std::thread thread_;
    std::atomic<bool> exit;
    void StartThread();
public:
    //Initialize the sim with a new PnC
    DracoSim(DracoInterface* interface, ScorpioInterface* arm_interface);
    //Start the simulator and visuals in a new thread
    void StartSim();
    //Stop the simulator and join the thread it was running in
    void StopSim();
};


