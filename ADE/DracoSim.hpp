#include <PnC/EnvInterface.hpp>

class DracoSim {
private:
    //dart::gui::osg::Viewer viewer;
    //Threading
    //std::thread thread_;
    //std::atomic<bool> exit;
    //void StartThread();
public:
    //Initialize the sim with a new PnC
    DracoSim();
    //Start the simulator and visuals in a new thread
    void StartSim(EnvInterface* interface, EnvInterface* arm_interface);
    //Stop the simulator and join the thread it was running in
    void StopSim();
};


