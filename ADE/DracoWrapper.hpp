#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/Scorpio2PnC/ScorpioInterface.hpp>
#include <ADE/DracoSim.hpp>

class DracoWrapper {
private:
    //PnC, sim, and robot members
    DracoInterface* interface_;
    ScorpioInterface* arm_interface_;
    Scorpio2Interface* arm2_interface_;
    DracoSim* simulator_;

    const std::string ARM1_NAME = "one";
    const int SLEEP_MILLIS = 50;

    //Logical Safety
    bool running_;

public:
    DracoWrapper();

    //Set up the DracoWrapper to create the PnC instance and pass it down into the sim and ros nodelet to centralize commands
    void Initialize();
    //Draco Walking Methods
    void SetWalkRawCommand(double ft_length, double r_ft_width, double l_ft_width,
                        double ori_inc, int num_step);
    void SetWalkXCommand(double x);
    void SetWalkYCommand(double y);
    void SetTurnCommand(double th);
    void SetWalkToRelativeCommand(double x, double y, double th);
    void SetHaltCommand();
    //Scorpio Manipulation Methods
    void SetMoveEndEffectorCommand(char *arm, double x, double y, double z);
    void SetCloseGripperCommand(char *arm);
    void SetOpenGripperCommand(char *arm);
    //Shutdown the sim and ros nodelet and destruct the PnC instance
    void Shutdown();
};


