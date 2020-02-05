#include <PnC/DracoPnC/DracoInterface.hpp>
#include <Configuration.h>
#include <Simulator/Dart/Draco/DracoWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <thread>

class DracoSim {
private:
    //Simulation Helpers
    osg::ref_ptr<DracoWorldNode> world_;
    dart::gui::osg::Viewer viewer;
    void displayJointFrames(const dart::simulation::WorldPtr& world,
                            const dart::dynamics::SkeletonPtr& robot);
    void addTargetFrame(const dart::simulation::WorldPtr& world);
    void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot);
    void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot);

    //Threading
    std::thread thread_;
    std::atomic<bool> exit;
    void StartThread();
public:
    //Initialize the sim with a new PnC
    DracoSim(DracoInterface* interface);
    //Start the simulator and visuals in a new thread
    void StartSim();
    //Stop the simulator and join the thread it was running in
    void StopSim();
};


